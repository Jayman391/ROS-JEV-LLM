// The assembly line world (SYSTEM_MODEL §1-§5): pure C++, no ROS, so test_line runs it directly.
//
// Discrete-event: step(dt) advances pallets, arm operations, jams and QA. An op is a duration
// plus a quality draw; every sensor is the hidden truth plus noise. Commands (release, route,
// operate, cancel, dispose) return "" when accepted, "done" when finished at once, otherwise why
// they were refused. Finished arm goals, QA-ready parts and parts that left the line queue up in
// `ended`, `qa_ready` and `left` for the ROS layer (line_sim) to drain.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace line {

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

inline std::vector<std::string> split(const std::string& s, char sep = ',') {
  std::vector<std::string> out;
  std::stringstream ss(s);
  for (std::string item; std::getline(ss, item, sep);)
    if (!item.empty()) out.push_back(item);
  return out;
}

// Gripping ops depend on grip quality; fasten's grip is the holding partner's.
inline bool gripping(const std::string& kind) {
  return kind == "pick" || kind == "insert" || kind == "fasten";
}

struct Params {
  std::vector<std::string> arms;
  std::vector<int64_t> arm_station;
  std::vector<std::string> arm_tools;   // "pick,screw"
  std::vector<std::string> products;
  std::vector<std::string> recipes;     // "pick,screw,weld,glue", one per product
  std::vector<std::string> op_kinds;
  std::vector<double> op_time_s;        // nominal duration per op kind
  std::vector<double> qa_sigma;         // QA reading noise per op kind
  int n_stations = 4;
  int buffer_cap = 2;
  double travel_s = 4.0, rework_lane_s = 20.0, qa_s = 5.0;
  double regrip_s = 3.0, tool_change_s = 25.0, q_min = 0.75;
  double jam_p0 = 0.01, jam_p1 = 0.04, jam_reset_min = 30.0, jam_reset_max = 90.0;
  double wear_rate_min = 0.006, wear_rate_max = 0.015, wear_fast_mult = 2.5;
  double p_grip_marginal = 0.10, p_grip_slip = 0.05, p_misalign = 0.06;
  int dropouts = 2;
  double dropout_s = 40.0, t_max_s = 1500.0;
  uint64_t seed = 0;
};

struct Op {
  std::string kind, arm, partner;   // route: primary arm (and the holding partner for fasten)
  std::string did;                  // who did it ("arm_d+arm_c")
  bool done = false;
  double q = NaN;                   // true quality
  int redo_of = -1;                 // a rework's redo: the recipe op it replaces
};

struct Part {
  int id = 0;
  std::string product;
  std::vector<Op> ops;              // recipe ops, then any rework redos
  size_t n_recipe = 0, k = 0;       // k: next op
  int reworks = 0;
  enum State { QUEUED, SLOT, TRANSIT, LANE } st = QUEUED;
  int stop = 0;                     // 0 feeder, 1..n stations, n+1 QA (TRANSIT: destination)
  int from = 0;                     // TRANSIT: origin stop
  double t_left = 0.0, t_total = 0.0;
  bool deferred = false, drawn = false, qa_sent = false;
  double grip = 1.0, grip_read = 1.0, partner_grip = 1.0, partner_grip_read = 1.0;
  double pose = 0.0, pose_read = 0.0;
  int regrips = 0;
  double released = 0.0;
};

struct Arm {
  std::string name;
  int station = 0;
  std::set<std::string> tools;
  double wear = 0.0, rate = 0.0, wear_est = 0.0;
  std::mt19937_64 rng;
  std::string state = "idle";       // idle | working | coop_wait | regrip | tool_change | jammed
  double busy_left = 0.0, busy_total = 0.0;
  int part = -1;
  std::string passive_of;           // fasten partner: the primary arm that drives the op
  double coop_wait = 0.0;
  bool will_jam = false;
  double jam_at = 0.0, jam_until = -1.0, jam_est = 1.0;
  double dropout_from = -1.0, dropout_to = -1.0;
  int ops_done = 0, since_change = 0, jams = 0;
};

struct GoalEnd {
  std::string arm;
  bool success;
  std::string message;
};

struct Outcome {                    // ground truth for a part leaving the line
  Part part;
  std::string outcome;              // shipped | scrapped | quarantined
  bool defective;
  double done;
};

struct QaItem {
  std::string kind, arm;
  double reading;
};

struct ArmView {                    // what the arm's own sensors report (ArmStatus)
  int part = -1, op = -1, regrips = 0, queue = 0;
  std::string kind, role, partner, partner_state;
  double grip = NaN, partner_grip = NaN, pose = NaN, wear = NaN, current = 1.0, jam_clear = 0.0;
  bool dropout = false;
};

class Line {
 public:
  explicit Line(Params params) : p(std::move(params)), rng(p.seed * 7919 + 17) {
    if (p.arms.size() != p.arm_station.size() || p.arms.size() != p.arm_tools.size())
      throw std::invalid_argument("arms, arm_station and arm_tools must have the same length");
    if (p.products.size() != p.recipes.size())
      throw std::invalid_argument("products and recipes must have the same length");
    if (p.op_kinds.size() != p.op_time_s.size() || p.op_kinds.size() != p.qa_sigma.size())
      throw std::invalid_argument("op_kinds, op_time_s and qa_sigma must have the same length");
    for (size_t i = 0; i < p.op_kinds.size(); ++i) {
      op_time[p.op_kinds[i]] = p.op_time_s[i];
      sigma[p.op_kinds[i]] = p.qa_sigma[i];
    }
    for (size_t i = 0; i < p.products.size(); ++i) {
      recipe[p.products[i]] = split(p.recipes[i]);
      for (const auto& k : recipe[p.products[i]])
        if (!op_time.count(k)) throw std::invalid_argument("recipe op without a time: " + k);
    }
    std::uniform_real_distribution<double> rate(p.wear_rate_min, p.wear_rate_max);
    for (size_t i = 0; i < p.arms.size(); ++i) {
      Arm a;
      a.name = p.arms[i];
      a.station = static_cast<int>(p.arm_station[i]);
      for (const auto& t : split(p.arm_tools[i])) a.tools.insert(t);
      a.rng.seed(p.seed * 1000 + i * 31 + 1);           // per-arm streams: comparable episodes
      a.rate = rate(a.rng);
      arms.push_back(a);
    }
    std::uniform_int_distribution<size_t> pick(0, arms.size() - 1);
    arms[pick(rng)].rate *= p.wear_fast_mult;            // one arm wears fast
    std::vector<size_t> idx(arms.size());
    for (size_t i = 0; i < idx.size(); ++i) idx[i] = i;
    std::shuffle(idx.begin(), idx.end(), rng);
    for (int d = 0; d < std::min<int>(p.dropouts, static_cast<int>(arms.size())); ++d) {
      Arm& a = arms[idx[d]];
      a.dropout_from = uniform(rng, 0.15, 0.7) * p.t_max_s;
      a.dropout_to = a.dropout_from + p.dropout_s;
    }
    queue.resize(p.n_stations + 2);
    slot.assign(p.n_stations + 2, -1);
    for (const auto& pr : p.products) released[pr] = shipped[pr] = scrapped[pr] = quarantined[pr] = 0;
  }

  // --- queries ------------------------------------------------------------------------------------

  int qa() const { return p.n_stations + 1; }
  bool feeder_free() const { return queue[0].empty(); }
  bool dropout(const Arm& a) const { return t >= a.dropout_from && t < a.dropout_to; }

  Arm* arm(const std::string& name) {
    for (auto& a : arms)
      if (a.name == name) return &a;
    return nullptr;
  }

  int station_of(const std::string& name) {
    Arm* a = arm(name);
    return a ? a->station : -1;
  }

  // pallets queued or in transit towards a stop (its buffer)
  int occupancy(int stop) const {
    int n = static_cast<int>(queue[stop].size());
    for (const auto& [id, pt] : parts)
      if (pt.st == Part::TRANSIT && pt.stop == stop) ++n;
    return n;
  }

  // the part in this arm's station slot with an op pending for it (primary or partner)
  Part* pending_for(const Arm& a) {
    int id = slot[a.station];
    if (id < 0) return nullptr;
    Part& pt = parts.at(id);
    if (pt.deferred || pt.k >= pt.ops.size()) return nullptr;
    const Op& op = pt.ops[pt.k];
    return op.arm == a.name || op.partner == a.name ? &pt : nullptr;
  }

  std::string where(const Part& pt) const {
    switch (pt.st) {
      case Part::TRANSIT: return "transit";
      case Part::LANE: return "lane";
      default:
        if (pt.stop == 0) return "feeder";
        if (pt.stop == qa()) return "QA";
        return "S" + std::to_string(pt.stop);
    }
  }

  ArmView view(Arm& a) {
    ArmView v;
    v.dropout = dropout(a);
    for (const auto& [id, pt] : parts)
      if (pt.k < pt.ops.size() && (pt.ops[pt.k].arm == a.name || pt.ops[pt.k].partner == a.name))
        ++v.queue;
    v.current = a.state == "jammed" ? 9.0 : a.state == "working" ? 4.0 : 1.0;
    if (a.state == "jammed") v.jam_clear = std::max(0.0, a.jam_until - t) * a.jam_est;
    Part* pt = a.part >= 0 && parts.count(a.part) ? &parts.at(a.part) : pending_for(a);
    if (!pt || pt->k >= pt->ops.size()) {
      v.wear = v.dropout ? NaN : a.wear_est;
      return v;
    }
    const Op& op = pt->ops[pt->k];
    v.part = pt->id;
    v.op = static_cast<int>(pt->k);
    v.kind = op.kind;
    v.regrips = pt->regrips;
    v.role = op.partner.empty() ? "solo" : op.arm == a.name ? "primary" : "partner";
    if (!op.partner.empty()) {
      v.partner = v.role == "primary" ? op.partner : op.arm;
      v.partner_state = arm(v.partner)->state;
    }
    if (!v.dropout) {
      if (v.role == "partner") v.grip = pt->partner_grip_read;
      else if (v.role == "primary") v.partner_grip = pt->partner_grip_read;
      else if (gripping(op.kind)) v.grip = pt->grip_read;
      v.pose = pt->pose_read;
      v.wear = a.wear_est;
    }
    return v;
  }

  // --- commands -----------------------------------------------------------------------------------

  std::string release(const std::string& product, const std::vector<std::string>& route, int* id) {
    if (!recipe.count(product)) return "unknown product " + product;
    if (!feeder_free()) return "feeder busy";
    const auto& r = recipe.at(product);
    if (route.size() != r.size()) return "route has " + std::to_string(route.size()) +
                                         " entries, recipe " + std::to_string(r.size());
    Part pt;
    pt.id = next_id++;
    pt.product = product;
    for (size_t i = 0; i < r.size(); ++i) {
      Op op;
      op.kind = r[i];
      std::string why = assign(op, route[i]);
      if (!why.empty()) return why;
      pt.ops.push_back(op);
    }
    pt.n_recipe = r.size();
    pt.released = t;
    parts[pt.id] = pt;
    queue[0].push_back(pt.id);
    ++released[product];
    if (id) *id = pt.id;
    return "";
  }

  std::string route(int id, size_t from, const std::vector<std::string>& entries) {
    if (!parts.count(id)) return "no part " + std::to_string(id);
    Part& pt = parts.at(id);
    if (from < pt.k) return "ops before " + std::to_string(pt.k) + " are done";
    if (from + entries.size() != pt.ops.size()) return "route must cover ops from..end";
    if (from == pt.k)
      for (const auto& a : arms)
        if (a.part == id && a.state != "idle") return "op in progress";
    for (size_t i = 0; i < entries.size(); ++i) {
      Op op = pt.ops[from + i];
      std::string why = assign(op, entries[i]);
      if (!why.empty()) return why;
      pt.ops[from + i] = op;
    }
    if (from == pt.k) {
      pt.drawn = false;                                  // new arm: fresh pre-op readings
      pt.regrips = 0;
    }
    return "";
  }

  std::string operate(const std::string& name, const std::string& kind, int part, int op_index) {
    Arm* a = arm(name);
    if (!a) return "rejected: no arm " + name;
    if (a->state == "jammed") return "rejected: jammed";
    if (a->state != "idle") return "rejected: busy (" + a->state + ")";
    if (kind == "tool_change") {
      start(*a, "tool_change", p.tool_change_s, -1);
      return "";
    }
    Part* pt = pending_for(*a);
    if (!pt || pt->id != part || static_cast<int>(pt->k) != op_index)
      return "rejected: no such op pending";
    Op& op = pt->ops[pt->k];
    if (kind == "defer") {
      pt->deferred = true;
      for (auto& b : arms)
        if (b.part == pt->id && b.state == "coop_wait") end(b, false, "deferred by partner");
      return "done";
    }
    if (kind == "regrip") {
      if (!gripping(op.kind) || (op.kind == "fasten" && op.partner != name))
        return "rejected: nothing to regrip";
      start(*a, "regrip", p.regrip_s, pt->id);
      return "";
    }
    if (kind != "op") return "rejected: unknown kind " + kind;
    if (op.partner.empty()) {
      begin_op(*a, *pt);
    } else {
      a->state = "coop_wait";
      a->part = pt->id;
      a->coop_wait = 0.0;
    }
    return "";
  }

  void cancel(const std::string& name) {
    Arm* a = arm(name);
    if (a && a->state == "coop_wait") end(*a, false, "cancelled");
  }

  std::string dispose(int id, const std::string& what, int op) {
    if (slot[qa()] != id || !parts.at(id).qa_sent) return "part " + std::to_string(id) +
                                                          " is not waiting at QA";
    Part& pt = parts.at(id);
    slot[qa()] = -1;
    if (what == "rework") {
      if (op < 0 || op >= static_cast<int>(pt.n_recipe)) {
        slot[qa()] = id;
        return "rework_op out of range";
      }
      Op redo;
      redo.kind = pt.ops[op].kind;
      redo.redo_of = op;
      pt.ops.push_back(redo);                            // unroutable until the supervisor routes it
      ++pt.reworks;
      pt.qa_sent = false;
      to_lane(pt);
      return "done";
    }
    std::string outcome = what == "pass" ? "shipped" : what == "scrap" ? "scrapped" :
                          what == "quarantine" ? "quarantined" : "";
    if (outcome.empty()) {
      slot[qa()] = id;
      return "unknown disposition " + what;
    }
    bool defective = false;
    for (size_t i = 0; i < pt.n_recipe; ++i) defective |= !(pt.ops[i].q >= p.q_min);
    (outcome == "shipped" ? shipped : outcome == "scrapped" ? scrapped : quarantined)[pt.product]++;
    left.push_back({pt, outcome, defective, t});
    parts.erase(id);
    return "done";
  }

  std::vector<QaItem> qa_readings(int id) {
    std::vector<QaItem> out;
    const Part& pt = parts.at(id);
    for (size_t i = 0; i < pt.n_recipe; ++i) {
      const Op& op = pt.ops[i];
      std::normal_distribution<double> n(0.0, sigma.at(op.kind));
      out.push_back({op.kind, op.did, std::clamp(op.q + n(rng), 0.0, 1.0)});
    }
    return out;
  }

  // --- time ---------------------------------------------------------------------------------------

  void step(double dt) {
    t += dt;
    for (auto& a : arms) {
      if (a.state == "jammed" && t >= a.jam_until) a.state = "idle";
      if (a.state == "coop_wait") a.coop_wait += dt;
      if (!a.passive_of.empty() || (a.state != "working" && a.state != "regrip" &&
                                    a.state != "tool_change"))
        continue;
      a.busy_left -= dt;
      if (a.state == "working" && a.will_jam && a.busy_total - a.busy_left >= a.jam_at) {
        jam(a);
      } else if (a.busy_left <= 0.0) {
        finish(a);
      }
    }
    for (int s = 1; s <= p.n_stations; ++s) start_coop(s);
    for (auto& [id, pt] : parts) {
      if (pt.st != Part::TRANSIT && pt.st != Part::LANE) continue;
      pt.t_left -= dt;
      if (pt.t_left > 0.0) continue;
      if (pt.st == Part::LANE) {
        pt.stop = 0;
        queue[0].push_front(id);                          // laps go ahead of new releases
      } else {
        queue[pt.stop].push_back(id);
      }
      pt.st = Part::QUEUED;
    }
    for (int s = qa(); s >= 1; --s) {                      // pull from buffers into free slots
      if (slot[s] >= 0 || queue[s].empty()) continue;
      Part& pt = parts.at(queue[s].front());
      queue[s].pop_front();
      slot[s] = pt.id;
      pt.st = Part::SLOT;
      pt.stop = s;
      if (s == qa()) pt.t_left = p.qa_s;
    }
    for (int s = p.n_stations; s >= 1; --s) {              // downstream first frees room upstream
      if (slot[s] < 0) continue;
      Part& pt = parts.at(slot[s]);
      if (pending_here(pt, s)) {
        if (!pt.drawn) draw(pt);
        continue;
      }
      if (busy_on(pt.id)) continue;
      int dest = next_stop(pt, s);
      if (dest == LANE) {
        slot[s] = -1;
        to_lane(pt);
      } else if (dest > s && occupancy(dest) < p.buffer_cap) {
        slot[s] = -1;
        to_transit(pt, s, dest);
      }
    }
    if (!queue[0].empty()) {                               // feeder: one pallet at a time
      Part& pt = parts.at(queue[0].front());
      int dest = next_stop(pt, 0);
      if (dest >= 1 && occupancy(dest) < p.buffer_cap) {
        queue[0].pop_front();
        to_transit(pt, 0, dest);
      }
    }
    if (slot[qa()] >= 0) {
      Part& pt = parts.at(slot[qa()]);
      pt.t_left -= dt;
      if (pt.t_left <= 0.0 && !pt.qa_sent) {
        pt.qa_sent = true;
        qa_ready.push_back(pt.id);
      }
    }
  }

  Params p;
  double t = 0.0;
  std::vector<Arm> arms;
  std::map<int, Part> parts;
  std::map<std::string, int> released, shipped, scrapped, quarantined;
  std::vector<GoalEnd> ended;   // drained by the ROS layer
  std::vector<int> qa_ready;
  std::vector<Outcome> left;

 private:
  static constexpr int LANE = -2, WAIT = -1;

  static double uniform(std::mt19937_64& g, double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(g);
  }
  static double normal(std::mt19937_64& g, double sd) {
    return std::normal_distribution<double>(0.0, sd)(g);
  }

  std::string assign(Op& op, const std::string& entry) {
    op.arm.clear();
    op.partner.clear();
    if (entry.empty()) return "";                          // unroutable: waits for a route
    auto names = split(entry, '+');
    if (names.size() > 2) return "bad route entry " + entry;
    for (const auto& n : names)
      if (!arm(n)) return "no arm " + n;
    op.arm = names[0];
    if (op.kind == "fasten") {
      if (names.size() != 2 || names[0] == names[1] ||
          station_of(names[0]) != station_of(names[1]))
        return "fasten needs screw+hold arms at one station: " + entry;
      if (!arm(names[0])->tools.count("screw") || !arm(names[1])->tools.count("hold"))
        return "fasten: " + names[0] + " must screw and " + names[1] + " hold";
      op.partner = names[1];
    } else if (names.size() != 1 || !arm(names[0])->tools.count(op.kind)) {
      return entry + " cannot " + op.kind;
    }
    return "";
  }

  bool pending_here(const Part& pt, int s) {
    if (pt.deferred || pt.k >= pt.ops.size()) return false;
    const Op& op = pt.ops[pt.k];
    return !op.arm.empty() && station_of(op.arm) == s;
  }

  bool busy_on(int id) const {
    for (const auto& a : arms)
      if (a.part == id && a.state != "idle") return true;
    return false;
  }

  // next stop from `from`: QA when done, the next op's station if downstream, else a lap
  int next_stop(const Part& pt, int from) {
    if (pt.k >= pt.ops.size()) return qa();
    const Op& op = pt.ops[pt.k];
    if (op.arm.empty()) return WAIT;
    int s = station_of(op.arm);
    return s > from ? s : LANE;
  }

  void to_transit(Part& pt, int from, int dest) {
    pt.st = Part::TRANSIT;
    pt.from = from;
    pt.stop = dest;
    pt.t_total = pt.t_left = p.travel_s * (dest - from);
    pt.deferred = pt.drawn = false;
    pt.regrips = 0;
  }

  void to_lane(Part& pt) {
    pt.st = Part::LANE;
    pt.from = pt.stop;
    pt.t_total = pt.t_left = p.rework_lane_s;
    pt.deferred = pt.drawn = false;
    pt.regrips = 0;
  }

  double draw_grip(std::mt19937_64& g) {
    double u = uniform(g, 0.0, 1.0);
    double base = u < p.p_grip_slip ? 0.3 : u < p.p_grip_slip + p.p_grip_marginal ? 0.6 : 0.92;
    return std::clamp(base + normal(g, 0.05), 0.0, 1.0);
  }

  // pre-op readings, drawn when an op becomes pending (and again on a regrip)
  void draw(Part& pt, const std::string& only = "") {
    const Op& op = pt.ops[pt.k];
    Arm& a = *arm(op.arm);
    if (only.empty() || only == op.arm) {
      pt.grip = draw_grip(a.rng);
      pt.grip_read = std::clamp(pt.grip + normal(a.rng, 0.04), 0.0, 1.0);
      pt.pose = uniform(a.rng, 0.0, 1.0) < p.p_misalign ? uniform(a.rng, 4.0, 7.0)
                                                         : std::abs(normal(a.rng, 1.0));
      pt.pose_read = std::abs(pt.pose + normal(a.rng, 0.3));
    }
    if (!op.partner.empty() && (only.empty() || only == op.partner)) {
      Arm& b = *arm(op.partner);
      pt.partner_grip = draw_grip(b.rng);
      pt.partner_grip_read = std::clamp(pt.partner_grip + normal(b.rng, 0.04), 0.0, 1.0);
    }
    pt.drawn = true;
  }

  void start(Arm& a, const std::string& state, double duration, int part) {
    a.state = state;
    a.busy_total = a.busy_left = duration;
    a.part = part;
    a.will_jam = false;
  }

  void begin_op(Arm& a, Part& pt) {
    double nominal = op_time.at(pt.ops[pt.k].kind);
    start(a, "working", std::max(0.5 * nominal, nominal * (1.0 + normal(a.rng, 0.1))), pt.id);
    a.will_jam = uniform(a.rng, 0.0, 1.0) < p.jam_p0 + p.jam_p1 * a.wear;
    a.jam_at = uniform(a.rng, 0.2, 0.8) * a.busy_total;
  }

  void start_coop(int s) {
    if (slot[s] < 0) return;
    Part& pt = parts.at(slot[s]);
    if (!pending_here(pt, s) || pt.ops[pt.k].partner.empty()) return;
    Arm& a = *arm(pt.ops[pt.k].arm);
    Arm& b = *arm(pt.ops[pt.k].partner);
    if (a.state != "coop_wait" || b.state != "coop_wait" || a.part != pt.id || b.part != pt.id)
      return;
    begin_op(a, pt);
    b.state = "working";
    b.passive_of = a.name;
  }

  void end(Arm& a, bool ok, const std::string& msg) {
    a.state = "idle";
    a.part = -1;
    a.passive_of.clear();
    a.coop_wait = 0.0;
    ended.push_back({a.name, ok, msg});
  }

  Arm* passive_partner(const Arm& a) {
    for (auto& b : arms)
      if (b.passive_of == a.name) return &b;
    return nullptr;
  }

  void jam(Arm& a) {
    Arm* b = passive_partner(a);
    end(a, false, "jam");
    a.state = "jammed";
    a.jam_until = t + uniform(a.rng, p.jam_reset_min, p.jam_reset_max);
    a.jam_est = std::exp(normal(a.rng, 0.3));
    ++a.jams;
    if (b) end(*b, false, "partner jammed");
  }

  void wear(Arm& a) {
    a.wear = std::min(1.0, a.wear + a.rate);
    a.wear_est = std::clamp(a.wear + normal(a.rng, 0.08), 0.0, 1.0);
    ++a.ops_done;
    ++a.since_change;
  }

  void finish(Arm& a) {
    if (a.state == "tool_change") {
      a.wear = 0.0;
      a.wear_est = std::clamp(normal(a.rng, 0.03) + 0.02, 0.0, 1.0);
      a.since_change = 0;
      return end(a, true, "done");
    }
    Part& pt = parts.at(a.part);
    if (a.state == "regrip") {
      draw(pt, pt.ops[pt.k].kind == "fasten" ? pt.ops[pt.k].partner : a.name);
      ++pt.regrips;
      return end(a, true, "done");
    }
    Op& op = pt.ops[pt.k];
    Arm* b = passive_partner(a);
    double w = b ? std::max(a.wear, b->wear) : a.wear;
    double g = op.kind == "fasten" ? pt.partner_grip : pt.grip;
    double q = 1.0 - 0.6 * w * w - (gripping(op.kind) ? 0.35 * (1.0 - g) : 0.0) -
               0.03 * std::max(0.0, pt.pose - 1.0) + normal(a.rng, 0.04);
    op.q = std::clamp(q, 0.0, 1.0);
    op.did = b ? a.name + "+" + b->name : a.name;
    op.done = true;
    if (op.redo_of >= 0) {
      pt.ops[op.redo_of].q = op.q;
      pt.ops[op.redo_of].did = op.did;
    }
    ++pt.k;
    pt.drawn = false;
    pt.regrips = 0;
    wear(a);
    end(a, true, "done");
    if (b) {
      wear(*b);
      end(*b, true, "done");
    }
  }

  std::mt19937_64 rng;                                    // line-level draws (QA, faults)
  std::map<std::string, double> op_time, sigma;
  std::map<std::string, std::vector<std::string>> recipe;
  std::vector<std::deque<int>> queue;                     // per stop: 0 feeder, 1..n, n+1 QA
  std::vector<int> slot;
  int next_id = 1;
};

}  // namespace line
