// flight_controller (B3): one per drone, in namespace /uavK. Deterministic; never asks a model.
//
// - Task action server (task): primitives TAKEOFF, GOTO, HOLD, SURVEY, ORBIT, FORMATION_SLOT, RTB,
//   LAND (SYSTEM_MODEL §5.1). A new goal preempts the current one, except LAND below 2 m. SURVEY
//   remembers its progress per `detail` (sector), so `continue` after an inspection resumes.
// - Safety (§7.1): H1-H5 filter every command (world.hpp); H6 energy reserve -> forced RTB;
//   H7 lost link -> hold, then RTB; H8 critical failure -> `critical` in the status (the agent's
//   require() turns it into land_now); a depleted battery lands in place.
// - Publishes DroneStatus on status and /swarm/status (5 Hz): the quantities the agent's masks use.
// - Drives the Gazebo X3 through MulticopterVelocityControl (gazebo/command/twist, body frame;
//   yaw is held at 0, so body = world up to small errors) and enable.
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "swarm_control/world.hpp"
#include "swarm_interfaces/action/task.hpp"
#include "swarm_interfaces/msg/drone_status.hpp"
#include "swarm_interfaces/msg/link_status.hpp"

namespace swarm_control
{

using Task = swarm_interfaces::action::Task;
using GoalHandle = rclcpp_action::ServerGoalHandle<Task>;
using swarm_interfaces::msg::DroneStatus;

static const std::set<std::string> PRIMITIVES = {
  "TAKEOFF", "GOTO", "HOLD", "SURVEY", "ORBIT", "FORMATION_SLOT", "RTB", "LAND"};

struct Job
{
  std::string prim{"LANDED"};          // a primitive, or IDLE (hover) / LANDED
  Task::Goal goal;
  std::shared_ptr<GoalHandle> gh;      // null: an internal job (reflex or idle)
  std::string why;                     // internal jobs: the reflex that started it
  std::vector<V3> wps;                 // SURVEY lawnmower
  size_t wp{0};
  V3 hold;                             // HOLD / IDLE point, ORBIT centre
  double t0{0}, swept{0}, last_angle{0}, settle{0};
  int stage{0};
  double progress{0}, eta{0};
};

class FlightController : public rclcpp::Node
{
public:
  FlightController()
  : Node("flight_controller")
  {
    p_ = load_params(*this);
    name_ = std::string(get_namespace()).substr(1);
    name_ = declare_parameter<std::string>("drone", name_.empty() ? "uav1" : name_);
    home_ = pad(p_, std::max(0, drone_index(name_)));

    auto sensor = rclcpp::SensorDataQoS();
    create_sub<nav_msgs::msg::Odometry>("odom", sensor, [this](nav_msgs::msg::Odometry::SharedPtr m) {
        const auto & q = m->pose.pose.orientation;
        pos_ = {m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z};
        // odometry twist is in the body frame: rotate to world by yaw
        yaw_ = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        const auto & t = m->twist.twist.linear;
        vel_ = {std::cos(yaw_) * t.x - std::sin(yaw_) * t.y,
          std::sin(yaw_) * t.x + std::cos(yaw_) * t.y, t.z};
        have_odom_ = true;
      });
    create_sub<sensor_msgs::msg::BatteryState>("battery", 10,
      [this](sensor_msgs::msg::BatteryState::SharedPtr m) {energy_frac_ = m->percentage;});
    create_sub<sensor_msgs::msg::LaserScan>("proximity", sensor,
      [this](sensor_msgs::msg::LaserScan::SharedPtr m) {
        ranges_.assign(m->ranges.begin(), m->ranges.end());
        a0_ = m->angle_min; da_ = m->angle_increment;
      });
    create_sub<swarm_interfaces::msg::LinkStatus>("link", 10,
      [this](swarm_interfaces::msg::LinkStatus::SharedPtr m) {link_q_ = m->quality;});
    create_sub<std_msgs::msg::String>("faults", 10, [this](std_msgs::msg::String::SharedPtr m) {
        faults_.clear();
        std::string s = m->data, tok;
        for (size_t i = 0; i <= s.size(); ++i) {
          if (i == s.size() || s[i] == ',') {if (!tok.empty()) {faults_.insert(tok);} tok.clear();}
          else {tok += s[i];}
        }
      });
    create_sub<geometry_msgs::msg::Vector3>("/env/wind", 10,
      [this](geometry_msgs::msg::Vector3::SharedPtr m) {wind_ = {m->x, m->y, 0};});
    create_sub<DroneStatus>("/swarm/status", 50, [this](DroneStatus::SharedPtr m) {
        if (m->drone != name_) {
          others_[m->drone] = {{m->position.x, m->position.y, m->position.z}, now_s(), m->airborne};
        }
      });

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("gazebo/command/twist", 10);
    enable_pub_ = create_publisher<std_msgs::msg::Bool>("enable", 10);
    status_pub_ = create_publisher<DroneStatus>("status", 10);
    swarm_pub_ = create_publisher<DroneStatus>("/swarm/status", 50);

    server_ = rclcpp_action::create_server<Task>(
      this, "task",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Task::Goal> g) {
        return accept(*g) ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
        rclcpp_action::GoalResponse::REJECT;
      },
      [](std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<GoalHandle> gh) {start(gh->get_goal()->primitive, *gh->get_goal(), gh);});

    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {step();});
    // the policy runs on sim time: the 50 Hz loop skips when the sim clock has not advanced
    status_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] {publish_status();});
    RCLCPP_INFO(get_logger(), "%s flight controller ready (home %.1f, %.1f)", name_.c_str(),
      home_.x, home_.y);
  }

private:
  template<class M, class Q, class F>
  void create_sub(const std::string & topic, const Q & qos, F && f)
  {
    subs_.push_back(create_subscription<M>(topic, qos, std::forward<F>(f)));
  }

  double now_s() {return get_clock()->now().seconds();}
  bool airborne() const {return pos_.z > 0.4;}
  bool homing() const {return job_.prim == "RTB" || job_.prim == "LAND";}

  bool accept(const Task::Goal & g)
  {
    if (!PRIMITIVES.count(g.primitive)) {return false;}
    if (job_.prim == "LAND" && airborne() && pos_.z < 2.0) {return false;}       // §5.1
    if (forced_ && g.primitive != "RTB" && g.primitive != "LAND") {return false;}
    if (!airborne() && margin_ <= 0 && g.primitive != "LAND") {return false;}   // no energy to fly
    return true;
  }

  void finish(bool ok, const std::string & msg)
  {
    if (job_.gh && job_.gh->is_active()) {
      auto r = std::make_shared<Task::Result>();
      r->success = ok;
      r->message = msg;
      if (ok) {job_.gh->succeed(r);} else {job_.gh->abort(r);}
    }
    job_.gh.reset();
  }

  void start(const std::string & prim, const Task::Goal & g, std::shared_ptr<GoalHandle> gh,
    const std::string & why = "")
  {
    if (job_.gh && job_.gh != gh) {finish(false, "preempted by " + prim);}
    Job j;
    j.prim = prim;
    j.goal = g;
    j.gh = gh;
    j.why = why;
    j.hold = pos_;
    j.t0 = now_s();
    if (prim == "SURVEY") {plan_survey(j);}
    if (prim == "ORBIT") {
      j.hold = {g.target.x, g.target.y, 0};
      j.last_angle = std::atan2(pos_.y - g.target.y, pos_.x - g.target.x);
    }
    job_ = j;
    if (airborne() || prim != "IDLE") {enable(true);}
    RCLCPP_INFO(get_logger(), "task %s%s%s", prim.c_str(),
      g.detail.empty() ? "" : (" " + g.detail).c_str(), why.empty() ? "" : (" (" + why + ")").c_str());
  }

  void plan_survey(Job & j)
  {
    const auto & a = j.goal.area;
    if (a.size() < 2) {return;}
    const double x0 = std::min(a[0].x, a[1].x), x1 = std::max(a[0].x, a[1].x);
    const double y0 = std::min(a[0].y, a[1].y), y1 = std::max(a[0].y, a[1].y);
    const double alt = j.goal.altitude > 0 ? j.goal.altitude : p_.alt_survey;
    const double lane = j.goal.lane_spacing > 0 ? j.goal.lane_spacing :
      2 * alt * std::tan(p_.det_fov_deg * M_PI / 360) * 0.8;
    const int n = std::max(1, static_cast<int>(std::ceil((y1 - y0) / lane)));
    std::vector<double> ys;
    for (int i = 0; i < n; ++i) {ys.push_back(y0 + (i + 0.5) * (y1 - y0) / n);}
    // start from the lane end nearest to the drone
    if (std::abs(pos_.y - ys.back()) < std::abs(pos_.y - ys.front())) {
      std::reverse(ys.begin(), ys.end());
    }
    bool fwd = std::abs(pos_.x - x0) <= std::abs(pos_.x - x1);
    for (double y : ys) {
      j.wps.push_back({fwd ? x0 : x1, y, alt});
      j.wps.push_back({fwd ? x1 : x0, y, alt});
      fwd = !fwd;
    }
    auto it = survey_done_.find(j.goal.detail);
    if (it != survey_done_.end() && it->second.size() == j.wps.size()) {
      j.wps = it->second;                     // same sector: keep its lane order, resume
      j.wp = survey_wp_[j.goal.detail];
    }
  }

  // velocity toward (x, y) at altitude alt, slowing near the target; climbs before moving when
  // close to the ground
  V3 go(double x, double y, double alt, double speed)
  {
    const double dx = x - pos_.x, dy = y - pos_.y, d = std::hypot(dx, dy);
    const double vh = std::min(speed, std::max(0.3, 0.6 * d));
    V3 v{d > 0.05 ? dx / d * vh : 0, d > 0.05 ? dy / d * vh : 0, std::clamp(alt - pos_.z, -3.0, 3.0)};
    if (pos_.z < 2.0 && alt > pos_.z + 1) {v.x = v.y = 0;}
    return v;
  }

  double hdist(double x, double y) const {return std::hypot(x - pos_.x, y - pos_.y);}

  void step()
  {
    if (!have_odom_) {return;}
    const double t = now_s();
    if (t == last_t_) {return;}                         // sim paused or clock not running
    dt_ = std::clamp(t - last_t_, 0.0, 0.1);
    last_t_ = t;
    reflexes_.clear();
    rtb_ = rtb_wh(p_, pos_, home_, wind_);
    margin_ = energy_margin(p_, energy_frac_ * p_.E_0, rtb_);
    link_lost_s_ = link_q_ < 0.1 ? link_lost_s_ + dt_ : 0.0;
    critical_ = faults_.count("motor") || (faults_.count("battery") && margin_ < -0.05);

    // --- reflexes that replace the task (H6, H7, depleted battery) ---
    forced_ = airborne() && margin_ <= 0;
    if (forced_) {reflexes_.push_back("forced_rtb");}
    if (airborne() && energy_frac_ <= 0.01 && job_.prim != "LAND") {
      finish(false, "battery depleted: landing in place");
      start("LAND", Task::Goal(), nullptr, "battery depleted");
    } else if (forced_ && !homing()) {
      finish(false, "forced RTB (H6: energy reserve)");
      start("RTB", Task::Goal(), nullptr, "H6 energy reserve");
    } else if (airborne() && link_lost_s_ >= p_.lost_link_rtb_s && !homing()) {
      finish(false, "lost link: RTB (H7)");
      start("RTB", Task::Goal(), nullptr, "H7 lost link");
    } else if (airborne() && link_lost_s_ >= p_.lost_link_hold_s && !homing() &&
      job_.why != "H7 lost link")
    {
      finish(false, "lost link: holding (H7)");
      start("HOLD", Task::Goal(), nullptr, "H7 lost link");
    }
    if (link_lost_s_ >= p_.lost_link_hold_s) {reflexes_.push_back("lost_link");}
    if (job_.gh && job_.gh->is_canceling()) {
      auto r = std::make_shared<Task::Result>();
      r->message = "cancelled";
      job_.gh->canceled(r);
      job_.gh.reset();
      start(airborne() ? "IDLE" : "LANDED", Task::Goal(), nullptr);
    }

    V3 v = primitive(t);
    if (job_.prim == "LANDED") {
      v_prev_ = {};
      twist_pub_->publish(geometry_msgs::msg::Twist());
      return;
    }
    // --- H1-H5 ---
    if (airborne()) {
      std::vector<V3> others;
      for (const auto & [n, o] : others_) {
        if (o.air && t - o.t < 2.0) {others.push_back(o.pos);}
      }
      const bool motor = faults_.count("motor") > 0;
      auto s = safety_filter(p_, pos_, v, ranges_, a0_, da_, others, motor ? 2.0 : p_.v_max_h,
          motor ? 0.5 : p_.v_max_up);
      v = s.v;
      reflexes_.insert(reflexes_.end(), s.reflexes.begin(), s.reflexes.end());
    }
    // acceleration limit
    const double dmax = p_.a_max * dt_;
    V3 dv = v - v_prev_;
    const double dh = dv.hnorm();
    if (dh > dmax) {dv.x *= dmax / dh; dv.y *= dmax / dh;}
    dv.z = std::clamp(dv.z, -2 * dmax, 2 * dmax);
    v = v_prev_ + dv;
    v_prev_ = v;
    geometry_msgs::msg::Twist tw;                     // world -> body (yaw)
    tw.linear.x = std::cos(yaw_) * v.x + std::sin(yaw_) * v.y;
    tw.linear.y = -std::sin(yaw_) * v.x + std::cos(yaw_) * v.y;
    tw.linear.z = v.z;
    tw.angular.z = std::clamp(-1.5 * std::atan2(std::sin(yaw_), std::cos(yaw_)),
        -p_.yaw_rate_max, p_.yaw_rate_max);
    twist_pub_->publish(tw);
    if (++enable_tick_ % 50 == 0) {enable(true);}
    feedback();
  }

  // The active primitive's desired velocity; completes jobs.
  V3 primitive(double t)
  {
    auto & j = job_;
    const auto & g = j.goal;
    const double alt_g = g.altitude > 0 ? g.altitude : p_.alt_transit;
    if (j.prim == "LANDED") {return {};}
    if (j.prim == "IDLE" || (j.prim == "HOLD")) {
      if (j.prim == "HOLD" && g.duration > 0 && t - j.t0 >= g.duration) {
        finish(true, "held");
        start("IDLE", Task::Goal(), nullptr);
      }
      j.progress = g.duration > 0 ? std::min(1.0, (t - j.t0) / g.duration) : 0.0;
      return go(j.hold.x, j.hold.y, std::max(j.hold.z, airborne() ? p_.alt_min : 0.0), 2.0);
    }
    if (j.prim == "TAKEOFF") {
      j.progress = std::clamp(pos_.z / alt_g, 0.0, 1.0);
      j.eta = std::abs(alt_g - pos_.z) / p_.v_max_up;
      if (std::abs(pos_.z - alt_g) < 0.5) {return complete("at altitude", pos_);}
      return {0.5 * (j.hold.x - pos_.x), 0.5 * (j.hold.y - pos_.y), std::clamp(alt_g - pos_.z, -2.0, 2.0)};
    }
    if (j.prim == "GOTO" || j.prim == "FORMATION_SLOT") {
      const double speed = g.speed > 0 ? g.speed : p_.v_transit;
      const double d = hdist(g.target.x, g.target.y);
      if (j.stage == 0) {j.settle = d; j.stage = 1;}
      j.progress = j.settle > 0 ? std::clamp(1 - d / j.settle, 0.0, 1.0) : 1.0;
      j.eta = d / speed;
      if (d < 1.5 && std::abs(pos_.z - alt_g) < 1.0) {
        return complete("arrived", {g.target.x, g.target.y, alt_g});
      }
      return go(g.target.x, g.target.y, alt_g, speed);
    }
    if (j.prim == "SURVEY") {
      if (j.wp >= j.wps.size()) {
        survey_wp_[g.detail] = j.wps.size();
        return complete("sector surveyed", pos_);
      }
      const auto & w = j.wps[j.wp];
      const double speed = g.speed > 0 ? g.speed : p_.v_survey;
      if (hdist(w.x, w.y) < 3.0 && std::abs(pos_.z - w.z) < 3.0) {
        ++j.wp;
        survey_done_[g.detail] = j.wps;
        survey_wp_[g.detail] = j.wp;
      }
      double left = j.wp < j.wps.size() ? hdist(j.wps[j.wp].x, j.wps[j.wp].y) : 0.0, total = 0;
      for (size_t i = 1; i < j.wps.size(); ++i) {
        const double seg = (j.wps[i] - j.wps[i - 1]).hnorm();
        total += seg;
        if (i > j.wp) {left += seg;}
      }
      j.progress = total > 0 ? std::clamp(1 - left / total, 0.0, 1.0) : 1.0;
      j.eta = left / speed;
      return j.wp < j.wps.size() ? go(j.wps[j.wp].x, j.wps[j.wp].y, j.wps[j.wp].z, speed) : V3{};
    }
    if (j.prim == "ORBIT") {
      const double r = g.radius > 0 ? g.radius : 6.0, alt = g.altitude > 0 ? g.altitude : p_.alt_inspect;
      const int laps = std::max(1, g.laps);
      const double cx = j.hold.x, cy = j.hold.y, d = hdist(cx, cy);
      const double a = std::atan2(pos_.y - cy, pos_.x - cx);
      if (j.stage == 0) {                             // approach the circle, then descend
        const double tx = cx + r * std::cos(a), ty = cy + r * std::sin(a);
        j.eta = hdist(tx, ty) / p_.v_transit + laps * 2 * M_PI * r / p_.v_inspect;
        if (hdist(tx, ty) < 2.0 && std::abs(pos_.z - alt) < 1.0) {j.stage = 1; j.last_angle = a;}
        return go(tx, ty, alt, hdist(tx, ty) > 20 ? p_.v_transit : p_.v_inspect);
      }
      double da = a - j.last_angle;
      da = std::atan2(std::sin(da), std::cos(da));
      j.swept += std::abs(da);
      j.last_angle = a;
      j.progress = std::min(1.0, j.swept / (2 * M_PI * laps));
      j.eta = (2 * M_PI * laps - j.swept) * r / p_.v_inspect;
      if (j.swept >= 2 * M_PI * laps) {
        return complete("orbit done", {pos_.x, pos_.y, alt});
      }
      const double nx = (pos_.x - cx) / std::max(d, 1e-3), ny = (pos_.y - cy) / std::max(d, 1e-3);
      return {-ny * p_.v_inspect + nx * (r - d), nx * p_.v_inspect + ny * (r - d),
        std::clamp(alt - pos_.z, -2.0, 2.0)};
    }
    if (j.prim == "RTB") {
      const double d = hdist(home_.x, home_.y);
      j.eta = d / p_.v_transit + pos_.z / p_.v_max_down;
      j.progress = 0.0;
      if (d < 2.0) {
        j.prim = "LAND";                              // same goal: RTB completes on the ground
        return {};
      }
      return go(home_.x, home_.y, p_.alt_transit, p_.v_transit);
    }
    if (j.prim == "LAND") {
      j.eta = pos_.z / p_.v_max_down;
      if (pos_.z < 0.15 && std::abs(vel_.z) < 0.2) {
        if ((j.settle += dt_) > 1.0) {
          enable(false);
          finish(true, "landed");
          start("LANDED", Task::Goal(), nullptr);
          return {};
        }
      } else {
        j.settle = 0;
      }
      if (j.stage == 0) {j.hold = pos_; j.stage = 1;}
      return {0.5 * (j.hold.x - pos_.x), 0.5 * (j.hold.y - pos_.y),
        pos_.z > 3 ? -p_.v_max_down : -0.6};
    }
    return {};
  }

  // finish the goal and hover at h
  V3 complete(const std::string & msg, const V3 & h)
  {
    finish(true, msg);
    start("IDLE", Task::Goal(), nullptr);
    job_.hold = h;
    return go(h.x, h.y, h.z, 1.0);
  }

  void enable(bool on)
  {
    std_msgs::msg::Bool b;
    b.data = on;
    enable_pub_->publish(b);
  }

  void feedback()
  {
    if (!job_.gh || ++fb_tick_ % 25) {return;}       // 2 Hz
    auto f = std::make_shared<Task::Feedback>();
    f->state = job_.prim;
    f->progress = job_.progress;
    f->eta_s = job_.eta;
    job_.gh->publish_feedback(f);
  }

  void publish_status()
  {
    if (!have_odom_) {return;}
    DroneStatus s;
    s.header.stamp = now();
    s.drone = name_;
    s.position.x = pos_.x; s.position.y = pos_.y; s.position.z = pos_.z;
    s.velocity.x = vel_.x; s.velocity.y = vel_.y; s.velocity.z = vel_.z;
    s.energy_frac = energy_frac_;
    s.energy_needed_frac = (rtb_ + p_.reserve * p_.E_0) / p_.E_0;
    s.margin = margin_;
    s.link_q = link_q_;
    s.link_lost_s = link_lost_s_;
    s.task = job_.prim;
    s.task_detail = job_.gh ? job_.goal.detail : job_.why;
    s.progress = job_.progress;
    s.eta_s = job_.eta;
    std::set<std::string> uniq(reflexes_.begin(), reflexes_.end());
    s.reflexes.assign(uniq.begin(), uniq.end());
    s.faults.assign(faults_.begin(), faults_.end());
    s.airborne = airborne();
    s.critical = critical_;
    double near_o = 1e9, near_d = 1e9;
    for (const auto & o : p_.obstacles) {
      const double d = std::hypot(pos_.x - o.x, pos_.y - o.y) - o.r;
      near_o = std::min(near_o, d);
      if (d < 1.0 && pos_.z > o.h) {s.over_obstacle = true;}
    }
    for (const auto & [n, o] : others_) {
      if (now_s() - o.t < 2.0) {near_d = std::min(near_d, (o.pos - pos_).norm());}
    }
    s.nearest_obstacle_m = near_o;
    s.nearest_drone_m = near_d;
    status_pub_->publish(s);
    swarm_pub_->publish(s);
  }

  struct Other
  {
    V3 pos;
    double t;
    bool air;
  };

  Params p_;
  std::string name_;
  V3 home_, pos_, vel_, wind_, v_prev_;
  double dt_{0.02}, yaw_{0}, energy_frac_{1.0}, link_q_{1.0}, link_lost_s_{0}, rtb_{0}, margin_{1}, last_t_{-1};
  double a0_{0}, da_{0};
  bool have_odom_{false}, forced_{false}, critical_{false};
  std::vector<double> ranges_;
  std::set<std::string> faults_;
  std::vector<std::string> reflexes_;
  std::map<std::string, Other> others_;
  std::map<std::string, std::vector<V3>> survey_done_;
  std::map<std::string, size_t> survey_wp_;
  Job job_;
  int enable_tick_{0}, fb_tick_{0};
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::Publisher<DroneStatus>::SharedPtr status_pub_, swarm_pub_;
  rclcpp_action::Server<Task>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_;
};

}  // namespace swarm_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swarm_control::FlightController>());
  rclcpp::shutdown();
  return 0;
}
