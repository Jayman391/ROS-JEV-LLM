// line_sim: the assembly line world as a ROS node (SYSTEM_MODEL §6). It owns /clock: every
// clock_step_s / real_time_factor wall seconds it advances line::Line by clock_step_s and
// publishes the new sim time, so every other node runs on sim time.
//
// ponytail: one node hosts the world and all arm action servers (single-threaded executor, no
// locks); split arm controllers out only if arms ever get real kinematics.
#include <array>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <line_interfaces/action/operate.hpp>
#include <line_interfaces/msg/arm_status.hpp>
#include <line_interfaces/msg/line_state.hpp>
#include <line_interfaces/msg/part_outcome.hpp>
#include <line_interfaces/msg/qa_reading.hpp>
#include <line_interfaces/srv/dispose.hpp>
#include <line_interfaces/srv/release.hpp>
#include <line_interfaces/srv/route.hpp>

#include "line_control/line.hpp"

using Operate = line_interfaces::action::Operate;
using GoalHandle = rclcpp_action::ServerGoalHandle<Operate>;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {

std::string fmt(double v, int prec = 2) {
  if (std::isnan(v)) return "n/a";
  std::ostringstream s;
  s << std::fixed << std::setprecision(prec) << v;
  return s.str();
}

KeyValue kv(const std::string& k, const std::string& v) {
  KeyValue m;
  m.key = k;
  m.value = v;
  return m;
}

}  // namespace

class LineSim : public rclcpp::Node {
 public:
  LineSim() : Node("line_sim") {
    line::Params p;
    p.arms = declare_parameter("arms", std::vector<std::string>{});
    p.arm_station = declare_parameter("arm_station", std::vector<int64_t>{});
    p.arm_tools = declare_parameter("arm_tools", std::vector<std::string>{});
    p.products = declare_parameter("products", std::vector<std::string>{});
    p.recipes = declare_parameter("recipes", std::vector<std::string>{});
    p.op_kinds = declare_parameter("op_kinds", std::vector<std::string>{});
    p.op_time_s = declare_parameter("op_time_s", std::vector<double>{});
    p.qa_sigma = declare_parameter("qa_sigma", std::vector<double>{});
    p.n_stations = declare_parameter("n_stations", p.n_stations);
    p.buffer_cap = declare_parameter("buffer_cap", p.buffer_cap);
    p.travel_s = declare_parameter("travel_s", p.travel_s);
    p.rework_lane_s = declare_parameter("rework_lane_s", p.rework_lane_s);
    p.qa_s = declare_parameter("qa_s", p.qa_s);
    p.regrip_s = declare_parameter("regrip_s", p.regrip_s);
    p.tool_change_s = declare_parameter("tool_change_s", p.tool_change_s);
    p.q_min = declare_parameter("q_min", p.q_min);
    p.jam_p0 = declare_parameter("jam_p0", p.jam_p0);
    p.jam_p1 = declare_parameter("jam_p1", p.jam_p1);
    p.jam_reset_min = declare_parameter("jam_reset_min", p.jam_reset_min);
    p.jam_reset_max = declare_parameter("jam_reset_max", p.jam_reset_max);
    p.wear_rate_min = declare_parameter("wear_rate_min", p.wear_rate_min);
    p.wear_rate_max = declare_parameter("wear_rate_max", p.wear_rate_max);
    p.wear_fast_mult = declare_parameter("wear_fast_mult", p.wear_fast_mult);
    p.p_grip_marginal = declare_parameter("p_grip_marginal", p.p_grip_marginal);
    p.p_grip_slip = declare_parameter("p_grip_slip", p.p_grip_slip);
    p.p_misalign = declare_parameter("p_misalign", p.p_misalign);
    p.dropouts = declare_parameter("dropouts", p.dropouts);
    p.dropout_s = declare_parameter("dropout_s", p.dropout_s);
    p.t_max_s = declare_parameter("t_max_s", p.t_max_s);
    p.seed = static_cast<uint64_t>(declare_parameter("seed", 0));
    step_s_ = declare_parameter("clock_step_s", 0.25);
    double rtf = declare_parameter("real_time_factor", 4.0);
    line_ = std::make_unique<line::Line>(p);

    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    state_pub_ = create_publisher<line_interfaces::msg::LineState>("/line/state", 10);
    qa_pub_ = create_publisher<line_interfaces::msg::QaReading>("/qa/reading", rclcpp::QoS(50).reliable());
    out_pub_ = create_publisher<line_interfaces::msg::PartOutcome>("/line/outcomes",
                                                                   rclcpp::QoS(200).reliable());
    marker_pub_ = create_publisher<MarkerArray>("/line/markers", 10);
    diag_pub_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
    for (auto& a : line_->arms) {
      status_pub_[a.name] = create_publisher<line_interfaces::msg::ArmStatus>("/" + a.name + "/status", 10);
      std::string name = a.name;
      servers_.push_back(rclcpp_action::create_server<Operate>(
          this, "/" + name + "/operate",
          [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Operate::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          },
          [this, name](std::shared_ptr<GoalHandle>) {
            // only a goal waiting for its fasten partner can be cancelled
            line::Arm* a = line_->arm(name);
            return a && a->state == "coop_wait" ? rclcpp_action::CancelResponse::ACCEPT
                                                : rclcpp_action::CancelResponse::REJECT;
          },
          [this, name](std::shared_ptr<GoalHandle> gh) { on_goal(name, gh); }));
    }
    release_srv_ = create_service<line_interfaces::srv::Release>(
        "/line/release", [this](const std::shared_ptr<line_interfaces::srv::Release::Request> rq,
                                std::shared_ptr<line_interfaces::srv::Release::Response> rs) {
          int id = -1;
          rs->message = line_->release(rq->product, rq->route, &id);
          rs->success = rs->message.empty();
          rs->part_id = id;
        });
    route_srv_ = create_service<line_interfaces::srv::Route>(
        "/line/route", [this](const std::shared_ptr<line_interfaces::srv::Route::Request> rq,
                              std::shared_ptr<line_interfaces::srv::Route::Response> rs) {
          rs->message = rq->from_op < 0 ? "from_op < 0" :
                        line_->route(rq->part_id, static_cast<size_t>(rq->from_op), rq->route);
          rs->success = rs->message.empty();
        });
    dispose_srv_ = create_service<line_interfaces::srv::Dispose>(
        "/line/dispose", [this](const std::shared_ptr<line_interfaces::srv::Dispose::Request> rq,
                                std::shared_ptr<line_interfaces::srv::Dispose::Response> rs) {
          rs->message = line_->dispose(rq->part_id, rq->disposition, rq->rework_op);
          rs->success = rs->message == "done";
          drain();
        });
    timer_ = create_wall_timer(std::chrono::duration<double>(step_s_ / rtf), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "line_sim: %zu arms, %d stations, %.2f s steps at %.1fx, seed %lu",
                line_->arms.size(), p.n_stations, step_s_, rtf, static_cast<unsigned long>(p.seed));
  }

 private:
  rclcpp::Time stamp() const { return rclcpp::Time(static_cast<int64_t>(line_->t * 1e9), RCL_ROS_TIME); }

  void on_goal(const std::string& arm, std::shared_ptr<GoalHandle> gh) {
    auto goal = gh->get_goal();
    auto result = std::make_shared<Operate::Result>();
    if (active_.count(arm)) {
      result->message = "rejected: a goal is active";
      return gh->abort(result);
    }
    std::string why = line_->operate(arm, goal->kind, goal->part_id, goal->op_index);
    if (why.empty()) {
      active_[arm] = gh;
      return;
    }
    result->success = why == "done";
    result->message = why;
    result->success ? gh->succeed(result) : gh->abort(result);
    drain();                 // a defer may have released the partner's waiting goal
  }

  void tick() {
    line_->step(step_s_);
    rosgraph_msgs::msg::Clock c;
    c.clock = stamp();
    clock_pub_->publish(c);
    drain();
    ++ticks_;
    for (auto& a : line_->arms) publish_status(a);
    if (ticks_ % 2 == 0) {
      publish_state();
      publish_markers();
    }
    if (ticks_ % 4 == 0) publish_diagnostics();
  }

  // finished goals, QA-ready parts and parts that left the line
  void drain() {
    for (const auto& e : line_->ended) {
      auto it = active_.find(e.arm);
      if (it == active_.end()) continue;
      auto result = std::make_shared<Operate::Result>();
      result->success = e.success;
      result->message = e.message;
      if (e.message == "cancelled" && it->second->is_canceling()) it->second->canceled(result);
      else if (e.success) it->second->succeed(result);
      else it->second->abort(result);
      active_.erase(it);
    }
    line_->ended.clear();
    for (int id : line_->qa_ready) {
      if (!line_->parts.count(id)) continue;
      const line::Part& pt = line_->parts.at(id);
      line_interfaces::msg::QaReading m;
      m.header.stamp = stamp();
      m.part_id = id;
      m.product = pt.product;
      m.reworks = pt.reworks;
      for (const auto& r : line_->qa_readings(id)) {
        m.op_kinds.push_back(r.kind);
        m.op_arms.push_back(r.arm);
        m.readings.push_back(r.reading);
      }
      qa_pub_->publish(m);
    }
    line_->qa_ready.clear();
    for (const auto& o : line_->left) {
      line_interfaces::msg::PartOutcome m;
      m.header.stamp = stamp();
      m.part_id = o.part.id;
      m.product = o.part.product;
      m.outcome = o.outcome;
      m.defective = o.defective;
      for (size_t i = 0; i < o.part.n_recipe; ++i) {
        m.op_kinds.push_back(o.part.ops[i].kind);
        m.op_arms.push_back(o.part.ops[i].did);
        m.quality.push_back(o.part.ops[i].q);
      }
      m.reworks = o.part.reworks;
      m.released_s = o.part.released;
      m.done_s = o.done;
      out_pub_->publish(m);
      RCLCPP_INFO(get_logger(), "part %d (%s) %s%s", o.part.id, o.part.product.c_str(),
                  o.outcome.c_str(), o.defective ? " [defective]" : "");
    }
    line_->left.clear();
  }

  void publish_status(line::Arm& a) {
    line::ArmView v = line_->view(a);
    line_interfaces::msg::ArmStatus m;
    m.header.stamp = stamp();
    m.arm = a.name;
    m.station = a.station;
    m.state = a.state;
    m.part_id = v.part;
    m.op_index = v.op;
    m.op_kind = v.kind;
    m.role = v.role;
    m.partner = v.partner;
    m.partner_state = v.partner_state;
    m.coop_wait_s = a.coop_wait;
    m.grip = v.grip;
    m.partner_grip = v.partner_grip;
    m.pose_mm = v.pose;
    m.wear_est = v.wear;
    m.current_a = v.current;
    m.dropout = v.dropout;
    m.jam_clear_est_s = v.jam_clear;
    m.regrips = v.regrips;
    m.queue = v.queue;
    m.ops_done = a.ops_done;
    m.ops_since_change = a.since_change;
    m.jams = a.jams;
    status_pub_.at(a.name)->publish(m);
  }

  void publish_state() {
    line_interfaces::msg::LineState m;
    m.header.stamp = stamp();
    m.t = line_->t;
    for (const auto& [id, pt] : line_->parts) {
      line_interfaces::msg::PartState s;
      s.id = id;
      s.product = pt.product;
      s.where = line_->where(pt);
      s.state = pt.st == line::Part::QUEUED ? "queued" : pt.st == line::Part::SLOT ? "slot" :
                pt.st == line::Part::TRANSIT ? "transit" : "lane";
      s.next_op = static_cast<int>(pt.k);
      s.n_ops = static_cast<int>(pt.ops.size());
      s.next_kind = pt.k < pt.ops.size() ? pt.ops[pt.k].kind : "";
      for (const auto& op : pt.ops)
        s.route.push_back(op.partner.empty() ? op.arm : op.arm + "+" + op.partner);
      s.reworks = pt.reworks;
      s.released_s = pt.released;
      m.parts.push_back(s);
    }
    for (int s = 1; s <= line_->qa(); ++s) m.buffer.push_back(line_->occupancy(s));
    m.buffer_cap = line_->p.buffer_cap;
    m.feeder_free = line_->feeder_free();
    for (const auto& pr : line_->p.products) {
      m.products.push_back(pr);
      m.released.push_back(line_->released[pr]);
      m.shipped.push_back(line_->shipped[pr]);
      m.scrapped.push_back(line_->scrapped[pr]);
      m.quarantined.push_back(line_->quarantined[pr]);
    }
    state_pub_->publish(m);
  }

  // --- RViz: stations along x, arms behind them, pallets on the belt, the lane in front ------------

  Marker marker(int id, int type, double x, double y, double z, double sx, double sy, double sz,
                std::array<float, 3> rgb) {
    Marker m;
    m.header.frame_id = "line";
    m.header.stamp = stamp();
    m.ns = "line";
    m.id = id;
    m.type = type;
    m.action = Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.pose.orientation.w = 1.0;
    m.scale.x = sx;
    m.scale.y = sy;
    m.scale.z = sz;
    m.color.r = rgb[0];
    m.color.g = rgb[1];
    m.color.b = rgb[2];
    m.color.a = 1.0;
    return m;
  }

  Marker text(int id, double x, double y, double z, const std::string& s) {
    Marker m = marker(id, Marker::TEXT_VIEW_FACING, x, y, z, 0, 0, 0.35, {1, 1, 1});
    m.text = s;
    return m;
  }

  void publish_markers() {
    const double dx = 3.0;
    MarkerArray ma;
    Marker clear;
    clear.action = Marker::DELETEALL;
    ma.markers.push_back(clear);
    int id = 0;
    for (int s = 1; s <= line_->qa(); ++s) {
      bool qa = s == line_->qa();
      ma.markers.push_back(marker(id++, Marker::CUBE, dx * s, 0, 0, 1.4, 1.4, 0.2,
                                  qa ? std::array<float, 3>{0.1f, 0.6f, 0.6f}
                                     : std::array<float, 3>{0.5f, 0.5f, 0.5f}));
      ma.markers.push_back(text(id++, dx * s, -1.0, 0.1, qa ? "QA" : "S" + std::to_string(s)));
    }
    ma.markers.push_back(marker(id++, Marker::CUBE, dx * (line_->qa() + 1) / 2.0, 0, -0.15,
                                dx * (line_->qa() + 1), 0.6, 0.05, {0.2f, 0.2f, 0.2f}));
    ma.markers.push_back(marker(id++, Marker::CUBE, dx * (line_->qa() + 1) / 2.0, -2.0, -0.15,
                                dx * (line_->qa() + 1), 0.4, 0.05, {0.35f, 0.25f, 0.1f}));
    std::map<int, int> at_station;
    for (auto& a : line_->arms) {
      int k = at_station[a.station]++;
      double x = dx * a.station + (k == 0 ? -0.4 : 0.4);
      static const std::map<std::string, std::array<float, 3>> color = {
          {"idle", {0.6f, 0.6f, 0.6f}}, {"working", {0.1f, 0.8f, 0.2f}},
          {"coop_wait", {0.95f, 0.85f, 0.1f}}, {"regrip", {0.95f, 0.55f, 0.1f}},
          {"tool_change", {0.2f, 0.4f, 0.95f}}, {"jammed", {0.95f, 0.1f, 0.1f}}};
      ma.markers.push_back(marker(id++, Marker::CUBE, x, 1.3, 0.6, 0.6, 0.6, 1.2,
                                  line_->dropout(a) ? std::array<float, 3>{0.7f, 0.2f, 0.9f}
                                                    : color.at(a.state)));
      ma.markers.push_back(text(id++, x, 1.3, 1.5, a.name.substr(4) + " " + a.state));
    }
    std::map<int, int> queued;
    for (const auto& [pid, pt] : line_->parts) {
      double x = 0, y = 0;
      if (pt.st == line::Part::SLOT) {
        x = dx * pt.stop;
      } else if (pt.st == line::Part::QUEUED) {
        x = dx * pt.stop - 0.6 * (++queued[pt.stop]);
      } else {
        double f = pt.t_total > 0 ? 1.0 - pt.t_left / pt.t_total : 1.0;
        if (pt.st == line::Part::TRANSIT) {
          x = dx * (pt.from + f * (pt.stop - pt.from));
        } else {
          x = dx * pt.from * (1.0 - f);
          y = -2.0;
        }
      }
      size_t prod = std::find(line_->p.products.begin(), line_->p.products.end(), pt.product) -
                    line_->p.products.begin();
      static const std::array<std::array<float, 3>, 3> pc = {
          {{0.9f, 0.25f, 0.2f}, {0.2f, 0.45f, 0.95f}, {0.2f, 0.8f, 0.35f}}};
      ma.markers.push_back(marker(id++, Marker::CUBE, x, y, 0.3, 0.45, 0.45, 0.35, pc[prod % 3]));
      ma.markers.push_back(text(id++, x, y, 0.75, pt.product + std::to_string(pid)));
    }
    marker_pub_->publish(ma);
  }

  void publish_diagnostics() {
    DiagnosticArray da;
    da.header.stamp = stamp();
    for (auto& a : line_->arms) {
      line::ArmView v = line_->view(a);
      DiagnosticStatus s;
      s.name = a.name;
      s.hardware_id = "S" + std::to_string(a.station);
      s.level = a.state == "jammed" ? DiagnosticStatus::ERROR :
                v.dropout ? DiagnosticStatus::WARN : DiagnosticStatus::OK;
      s.message = a.state + (v.dropout ? " (sensor dropout)" : "");
      s.values = {kv("station", "S" + std::to_string(a.station)), kv("state", a.state),
                  kv("part", v.part < 0 ? "-" : std::to_string(v.part)),
                  kv("op", v.kind.empty() ? "-" : v.kind), kv("wear_est", fmt(v.wear)),
                  kv("queue", std::to_string(v.queue)), kv("ops", std::to_string(a.ops_done)),
                  kv("since_tool_change", std::to_string(a.since_change)),
                  kv("jams", std::to_string(a.jams))};
      da.status.push_back(s);
    }
    DiagnosticStatus s;
    s.name = "line_sim";
    s.level = DiagnosticStatus::OK;
    int ship = 0, scrap = 0, quar = 0, rel = 0;
    for (const auto& pr : line_->p.products) {
      rel += line_->released[pr];
      ship += line_->shipped[pr];
      scrap += line_->scrapped[pr];
      quar += line_->quarantined[pr];
    }
    s.message = "t " + fmt(line_->t, 0) + " s, WIP " + std::to_string(line_->parts.size());
    s.values = {kv("t_s", fmt(line_->t, 0)), kv("wip", std::to_string(line_->parts.size())),
                kv("released", std::to_string(rel)), kv("shipped", std::to_string(ship)),
                kv("scrapped", std::to_string(scrap)), kv("quarantined", std::to_string(quar))};
    da.status.push_back(s);
    diag_pub_->publish(da);
  }

  std::unique_ptr<line::Line> line_;
  double step_s_ = 0.25;
  uint64_t ticks_ = 0;
  std::map<std::string, std::shared_ptr<GoalHandle>> active_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<line_interfaces::msg::LineState>::SharedPtr state_pub_;
  rclcpp::Publisher<line_interfaces::msg::QaReading>::SharedPtr qa_pub_;
  rclcpp::Publisher<line_interfaces::msg::PartOutcome>::SharedPtr out_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
  std::map<std::string, rclcpp::Publisher<line_interfaces::msg::ArmStatus>::SharedPtr> status_pub_;
  std::vector<rclcpp_action::Server<Operate>::SharedPtr> servers_;
  rclcpp::Service<line_interfaces::srv::Release>::SharedPtr release_srv_;
  rclcpp::Service<line_interfaces::srv::Route>::SharedPtr route_srv_;
  rclcpp::Service<line_interfaces::srv::Dispose>::SharedPtr dispose_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineSim>());
  rclcpp::shutdown();
  return 0;
}
