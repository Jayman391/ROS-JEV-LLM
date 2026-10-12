// line_gz: the Gazebo Fortress puppet of the assembly line. Display only: line_sim decides
// everything and nothing flows back. Talks ign-transport directly (no ros_gz_bridge: Pose_V and
// Visual have no ROS service types), so it must share Gazebo's network namespace.
//
//   /line/markers (ns "pallet", id = part id)  -> pallet pool poses (set_pose_vector) + colour
//   /<arm>/status                               -> arm column colour (visual_config) + joint pose
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>
#include <line_interfaces/msg/arm_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using line_interfaces::msg::ArmStatus;
using visualization_msgs::msg::MarkerArray;
using Rgb = std::array<double, 3>;

namespace {

// the RViz arm colours (line_sim publish_markers)
const std::map<std::string, Rgb> kStateColor = {
    {"idle", {0.6, 0.6, 0.6}},        {"working", {0.1, 0.8, 0.2}},
    {"coop_wait", {0.95, 0.85, 0.1}}, {"regrip", {0.95, 0.55, 0.1}},
    {"tool_change", {0.2, 0.4, 0.95}}, {"jammed", {0.95, 0.1, 0.1}}};
const Rgb kDropout = {0.7, 0.2, 0.9};
const char* kJoints[3] = {"j0", "j1", "j2"};

struct Arm {
  std::string state = "idle";
  Rgb color{-1, -1, -1};
  std::array<double, 3> q{0, 0, 0};  // the targets sent so far (rate-limited)
  std::vector<ignition::transport::Node::Publisher> joint;
};

}  // namespace

class LineGz : public rclcpp::Node {
 public:
  LineGz() : Node("line_gz") {
    world_ = declare_parameter("world", "line");
    z0_ = declare_parameter("z_offset", 0.3);
    pool_ = declare_parameter("pallet_pool", 16);
    park_z_ = declare_parameter("park_z", -3.0);
    speed_ = declare_parameter("joint_speed", 2.0);   // rad per wall s
    for (const char* k : {"home", "reach", "lift", "tool"})
      pose_[k] = declare_parameter(std::string("pose_") + k, std::vector<double>{0, 0, 0});
    for (const auto& name : declare_parameter("arms", std::vector<std::string>{})) {
      Arm& a = arms_[name];
      for (const char* j : kJoints)
        a.joint.push_back(gz_.Advertise<ignition::msgs::Double>(
            "/model/" + name + "/joint/" + j + "/0/cmd_pos"));
      subs_.push_back(create_subscription<ArmStatus>(
          "/" + name + "/status", 10,
          [this, name](ArmStatus::SharedPtr m) { status(name, *m); }));
    }
    for (int i = 0; i < pool_; ++i) free_.insert(i);
    slot_color_.assign(pool_, Rgb{-1, -1, -1});
    markers_ = create_subscription<MarkerArray>(
        "/line/markers", 10, [this](MarkerArray::SharedPtr m) { pallets(*m); });
    // wall time: the arms animate even while the sim clock is paused
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { animate(); });
    RCLCPP_INFO(get_logger(), "puppeting %zu arms and %d pallets in Gazebo world '%s'",
                arms_.size(), pool_, world_.c_str());
  }

 private:
  // --- pallets: a pool of models, one per part on the line; the rest parked under the floor ---
  void pallets(const MarkerArray& ma) {
    ignition::msgs::Pose_V req;
    std::set<int> seen;
    for (const auto& m : ma.markers) {
      if (m.ns != "pallet") continue;
      seen.insert(m.id);
      auto it = slot_.find(m.id);
      if (it == slot_.end()) {
        if (free_.empty()) continue;   // ponytail: pool exhausted, the extra part goes unseen
        it = slot_.emplace(m.id, *free_.begin()).first;
        free_.erase(free_.begin());
      }
      int s = it->second;
      set_pose(req.add_pose(), s, m.pose.position.x, m.pose.position.y, m.pose.position.z + z0_);
      Rgb c{m.color.r, m.color.g, m.color.b};
      if (c != slot_color_[s]) {
        slot_color_[s] = c;
        recolor("pallet_" + std::to_string(s) + "_link", "v", c);
      }
    }
    for (auto it = slot_.begin(); it != slot_.end();) {   // parts that left the line
      if (seen.count(it->first)) { ++it; continue; }
      set_pose(req.add_pose(), it->second, 0, 0, park_z_);
      free_.insert(it->second);
      it = slot_.erase(it);
    }
    if (req.pose_size() == 0) return;
    gz_.Request("/world/" + world_ + "/set_pose_vector", req,
                ignore_);
  }

  void set_pose(ignition::msgs::Pose* p, int slot, double x, double y, double z) {
    p->set_name("pallet_" + std::to_string(slot));
    p->mutable_position()->set_x(x);
    p->mutable_position()->set_y(y);
    p->mutable_position()->set_z(z);
    p->mutable_orientation()->set_w(1.0);
  }

  void recolor(const std::string& parent, const std::string& visual, const Rgb& c) {
    ignition::msgs::Visual v;
    v.set_parent_name(parent);
    v.set_name(visual);
    for (auto* col : {v.mutable_material()->mutable_ambient(), v.mutable_material()->mutable_diffuse()}) {
      col->set_r(c[0]);
      col->set_g(c[1]);
      col->set_b(c[2]);
      col->set_a(1.0);
    }
    gz_.Request("/world/" + world_ + "/visual_config", v,
                ignore_);
  }

  // --- arms: column colour = state; joints play a pose per state ---------------------------------
  void status(const std::string& name, const ArmStatus& m) {
    Arm& a = arms_.at(name);
    a.state = m.state;
    auto it = kStateColor.find(m.state);
    Rgb c = m.dropout ? kDropout : it != kStateColor.end() ? it->second : Rgb{0.6, 0.6, 0.6};
    if (c != a.color) {
      a.color = c;
      recolor(name + "_column", "v", c);
    }
  }

  void animate() {
    double t = now_s();
    double step = speed_ * 0.05;
    for (auto& [name, a] : arms_) {
      if (a.state == "jammed") continue;   // frozen where it stopped
      std::array<double, 3> q;
      if (a.state == "working") {           // on the part, busy
        q = pose("reach");
        q[0] += 0.06 * std::sin(2 * M_PI * t / 0.9);
        q[2] += 0.10 * std::sin(2 * M_PI * t / 0.6);
      } else if (a.state == "coop_wait") {
        q = pose("reach");
      } else if (a.state == "regrip") {     // lift and set down again
        q = std::fmod(t, 1.0) < 0.5 ? pose("lift") : pose("reach");
      } else if (a.state == "tool_change") {
        q = pose("tool");
      } else {
        q = pose("home");
      }
      for (int j = 0; j < 3; ++j) {
        a.q[j] += std::clamp(q[j] - a.q[j], -step, step);
        ignition::msgs::Double d;
        d.set_data(a.q[j]);
        a.joint[j].Publish(d);
      }
    }
  }

  std::array<double, 3> pose(const std::string& k) const {
    const auto& v = pose_.at(k);
    return {v.at(0), v.at(1), v.at(2)};
  }

  double now_s() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  ignition::transport::Node gz_;
  // async requests take the reply callback by non-const reference; replies are not needed
  std::function<void(const ignition::msgs::Boolean&, const bool)> ignore_ =
      [](const ignition::msgs::Boolean&, const bool) {};
  std::string world_;
  double z0_, park_z_, speed_;
  int pool_;
  std::map<std::string, std::vector<double>> pose_;
  std::map<std::string, Arm> arms_;
  std::map<int, int> slot_;   // part id -> pool slot
  std::set<int> free_;
  std::vector<Rgb> slot_color_;
  std::vector<rclcpp::Subscription<ArmStatus>::SharedPtr> subs_;
  rclcpp::Subscription<MarkerArray>::SharedPtr markers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineGz>());
  rclcpp::shutdown();
  return 0;
}
