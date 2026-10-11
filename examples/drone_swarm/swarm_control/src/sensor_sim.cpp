// sensor_sim (B2): one per drone (namespace /uavK). Simulated sensors Gazebo doesn't provide
// without a GPU, computed from ground truth plus noise (SYSTEM_MODEL §3.1, §4):
//   battery    (sensor_msgs/BatteryState, 2 Hz)  energy integrated from the power model; x3 drain
//                                                under battery_fault
//   proximity  (sensor_msgs/LaserScan, 10 Hz)    16 horizontal beams, 20 m, ray-cast vs obstacles
//   detections (DetectionArray, 2 Hz)            downward camera + classifier model, local tracks
//   link       (LinkStatus, 1 Hz)                q to the base, or via the best relay hop
//   faults     (std_msgs/String, 1 Hz)           what onboard fault detection reports (5 s delay)
// Only this node, environment and mission_scorer read /env/ground_truth.
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include "swarm_control/world.hpp"
#include "swarm_interfaces/msg/detection_array.hpp"
#include "swarm_interfaces/msg/drone_status.hpp"
#include "swarm_interfaces/msg/ground_truth.hpp"
#include "swarm_interfaces/msg/link_status.hpp"

namespace swarm_control
{

using swarm_interfaces::msg::Detection;
using swarm_interfaces::msg::DetectionArray;
using swarm_interfaces::msg::DroneStatus;
using swarm_interfaces::msg::GroundTruth;

static const std::vector<std::string> CLASSES = {"survivor", "vehicle", "debris", "animal",
  "nothing"};
// Confusion rows (true class -> score mass), at survey altitude (30 m) and inspection (10 m).
// Survey: ~60% on the true class; survivor<->animal and debris<->nothing confused (§4.6).
static const double SURVEY[5][5] = {
  {0.60, 0.05, 0.05, 0.25, 0.05}, {0.05, 0.65, 0.15, 0.05, 0.10}, {0.05, 0.10, 0.55, 0.05, 0.25},
  {0.25, 0.05, 0.05, 0.60, 0.05}, {0.20, 0.10, 0.25, 0.15, 0.30}};
static const double INSPECT[5][5] = {
  {0.90, 0.02, 0.02, 0.04, 0.02}, {0.02, 0.90, 0.04, 0.02, 0.02}, {0.02, 0.03, 0.88, 0.02, 0.05},
  {0.04, 0.02, 0.02, 0.90, 0.02}, {0.05, 0.03, 0.07, 0.05, 0.80}};
constexpr double DIRICHLET_ALPHA = 30.0;   // concentration: score noise around the row
constexpr double GPS_SIGMA = 1.5;
constexpr double PROX_RANGE = 20.0;
constexpr int BEAMS = 16;

class SensorSim : public rclcpp::Node
{
public:
  SensorSim()
  : Node("sensor_sim")
  {
    p_ = load_params(*this);
    name_ = std::string(get_namespace()).substr(1);
    name_ = declare_parameter<std::string>("drone", name_.empty() ? "uav1" : name_);
    rng_.seed(static_cast<unsigned>(p_.seed * 1000 + std::max(0, drone_index(name_))));
    energy_ = p_.E_0;

    subs_.push_back(create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr m) {
        const auto & q = m->pose.pose.orientation;
        const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        const auto & t = m->twist.twist.linear;
        pos_ = {m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z};
        vel_ = {std::cos(yaw) * t.x - std::sin(yaw) * t.y, std::sin(yaw) * t.x + std::cos(yaw) * t.y,
          t.z};
        have_odom_ = true;
      }));
    subs_.push_back(create_subscription<GroundTruth>("/env/ground_truth",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](GroundTruth::SharedPtr m) {gt_ = m;}));
    subs_.push_back(create_subscription<DroneStatus>("/swarm/status", 50,
      [this](DroneStatus::SharedPtr m) {
        if (m->drone != name_) {
          peers_[m->drone] = {{m->position.x, m->position.y, m->position.z}, m->link_q,
            now_s(), m->airborne};
        }
      }));
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery", 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("proximity", rclcpp::SensorDataQoS());
    det_pub_ = create_publisher<DetectionArray>("detections", 10);
    link_pub_ = create_publisher<swarm_interfaces::msg::LinkStatus>("link", 10);
    fault_pub_ = create_publisher<std_msgs::msg::String>("faults", 10);
    timers_.push_back(create_wall_timer(std::chrono::milliseconds(100), [this] {fast();}));
    timers_.push_back(create_wall_timer(std::chrono::milliseconds(500), [this] {detect();}));
    timers_.push_back(create_wall_timer(std::chrono::seconds(1), [this] {slow();}));
  }

private:
  double now_s() {return get_clock()->now().seconds();}

  bool fault(const std::string & kind, double delay = 0.0)
  {
    if (!gt_) {return false;}
    const double t = now_s();
    for (size_t k = 0; k < gt_->fault_kinds.size(); ++k) {
      if (gt_->fault_drones[k] == name_ && gt_->fault_kinds[k] == kind &&
        t >= gt_->fault_start_s[k] + delay)
      {
        return true;
      }
    }
    return false;
  }

  V3 wind() const {return gt_ ? V3{gt_->wind.x, gt_->wind.y, 0} : V3{};}

  // 10 Hz: battery integration + proximity scan
  void fast()
  {
    const double t = now_s();
    if (!have_odom_ || t <= last_t_) {return;}
    const double dt = last_t_ < 0 ? 0.0 : std::min(0.5, t - last_t_);
    last_t_ = t;
    if (pos_.z > 0.3) {
      energy_ -= power_w(p_, vel_, wind()) * (fault("battery_fault") ? 3.0 : 1.0) * dt / 3600;
      energy_ = std::max(0.0, energy_);
    }
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = now();
    scan.header.frame_id = name_ + "/world_aligned";      // beams in world orientation
    scan.angle_min = 0.0;
    scan.angle_increment = 2 * M_PI / BEAMS;
    scan.angle_max = scan.angle_increment * (BEAMS - 1);
    scan.range_min = 0.1;
    scan.range_max = PROX_RANGE;
    std::normal_distribution<double> noise(0.0, 0.1);
    for (int i = 0; i < BEAMS; ++i) {
      double r = std::numeric_limits<double>::infinity();
      for (const auto & o : p_.obstacles) {
        r = std::min(r, ray_cylinder(pos_.x, pos_.y, pos_.z, i * scan.angle_increment, o));
      }
      scan.ranges.push_back(r <= PROX_RANGE ? static_cast<float>(std::max(0.0, r + noise(rng_))) :
        std::numeric_limits<float>::infinity());
    }
    scan_pub_->publish(scan);
  }

  // 2 Hz: detector (and the battery reading)
  void detect()
  {
    if (!have_odom_) {return;}
    sensor_msgs::msg::BatteryState b;
    b.header.stamp = now();
    b.percentage = static_cast<float>(energy_ / p_.E_0);
    b.voltage = static_cast<float>(14.8 * (0.85 + 0.15 * energy_ / p_.E_0));
    b.present = true;
    battery_pub_->publish(b);

    if (!gt_ || pos_.z < 2.0) {return;}
    const double h = pos_.z, speed = vel_.hnorm();
    const double r = h * std::tan(p_.det_fov_deg * M_PI / 360);
    const double sigma = (0.05 * h + GPS_SIGMA) * (fault("gps_degraded") ? 3.0 : 1.0);
    std::uniform_real_distribution<double> U(0.0, 1.0);
    std::normal_distribution<double> N(0.0, sigma);
    DetectionArray out;
    out.header.stamp = now();
    out.drone = name_;
    for (size_t i = 0; i < gt_->target_ids.size(); ++i) {
      const auto & tp = gt_->target_positions[i];
      if (std::hypot(tp.x - pos_.x, tp.y - pos_.y) > r) {continue;}
      const double pd = p_.det_p0 * std::exp(-std::pow(h - p_.det_h_opt, 2) /
          (2 * p_.det_sigma_h * p_.det_sigma_h)) * std::exp(-speed / p_.det_v_blur) *
        gt_->visibility;
      if (U(rng_) >= pd) {continue;}
      const int cls = std::find(CLASSES.begin(), CLASSES.end(), gt_->target_classes[i]) -
        CLASSES.begin();
      out.detections.push_back(sighting(std::min(cls, 4), tp.x + N(rng_), tp.y + N(rng_), h, speed));
    }
    // false positives: Poisson(lambda * cells swept since the last tick), inside the AO
    const bool in_ao = pos_.x > p_.ao[0] && pos_.x < p_.ao[1] && pos_.y > p_.ao[2] &&
      pos_.y < p_.ao[3];
    if (in_ao && h > 5) {
      const double cells = 2 * r * std::max(speed, 0.5) * 0.5 / 100.0;
      std::poisson_distribution<int> fp(p_.det_lambda_fp * cells);
      for (int k = fp(rng_); k > 0; --k) {
        const double a = U(rng_) * 2 * M_PI, rr = r * std::sqrt(U(rng_));
        out.detections.push_back(sighting(4, pos_.x + rr * std::cos(a), pos_.y + rr * std::sin(a),
          h, speed));
      }
    }
    if (!out.detections.empty()) {det_pub_->publish(out);}
  }

  Detection sighting(int cls, double x, double y, double h, double speed)
  {
    const double w = std::clamp((h - p_.alt_inspect) / (p_.alt_survey - p_.alt_inspect), 0.0, 1.0);
    Detection d;
    double sum = 0;
    for (int k = 0; k < 5; ++k) {
      const double m = (1 - w) * INSPECT[cls][k] + w * SURVEY[cls][k];
      std::gamma_distribution<double> g(DIRICHLET_ALPHA * m, 1.0);
      d.scores.push_back(g(rng_) + 1e-9);
      sum += d.scores.back();
    }
    for (auto & s : d.scores) {s /= sum;}
    d.classes = CLASSES;
    d.position.x = x;
    d.position.y = y;
    d.altitude = h;
    d.speed = speed;
    // local data association: within 5 m of a known local track
    int best = -1;
    double bd = 5.0;
    for (size_t k = 0; k < tracks_.size(); ++k) {
      const double dd = std::hypot(tracks_[k].x - x, tracks_[k].y - y);
      if (dd < bd) {bd = dd; best = static_cast<int>(k);}
    }
    if (best < 0) {
      tracks_.push_back({x, y, 0});
      best = static_cast<int>(tracks_.size()) - 1;
    }
    auto & t = tracks_[best];
    t.n += 1;
    t.x += (x - t.x) / t.n;
    t.y += (y - t.y) / t.n;
    d.track_id = name_ + "-" + std::to_string(best + 1);
    return d;
  }

  // 1 Hz: link quality (base or the best relay hop) and detected faults
  void slow()
  {
    if (!have_odom_) {return;}
    const double t = now_s();
    auto blacked = [&](const V3 & q) {
        if (!gt_) {return false;}
        for (size_t k = 0; k < gt_->fault_kinds.size(); ++k) {
          if (gt_->fault_kinds[k] != "comms_blackout" || t < gt_->fault_start_s[k]) {continue;}
          for (const auto & b : gt_->blackouts) {
            if (std::hypot(q.x - b.x, q.y - b.y) < b.z) {return true;}
          }
        }
        return false;
      };
    swarm_interfaces::msg::LinkStatus l;
    l.header.stamp = now();
    l.drone = name_;
    l.distance = pos_.norm();
    l.quality = link_quality(p_, l.distance);
    l.via = "base";
    for (const auto & [n, peer] : peers_) {
      if (!peer.air || t - peer.t > 3.0) {continue;}
      const double hop = (peer.pos - pos_).norm();
      const double q = std::min(link_quality(p_, hop), peer.q);
      if (q > l.quality) {l.quality = q; l.via = n; l.distance = hop;}
    }
    if (blacked(pos_)) {l.quality = 0.0;}
    link_pub_->publish(l);

    std_msgs::msg::String f;                // onboard detectors notice faults after ~5 s
    for (const auto & [kind, label] : std::map<std::string, std::string>{
        {"battery_fault", "battery"}, {"gps_degraded", "gps"}, {"motor_degraded", "motor"}})
    {
      if (fault(kind, 5.0)) {f.data += (f.data.empty() ? "" : ",") + label;}
    }
    fault_pub_->publish(f);
  }

  struct Peer
  {
    V3 pos;
    double q, t;
    bool air;
  };
  struct LocalTrack
  {
    double x, y;
    int n;
  };

  Params p_;
  std::string name_;
  std::mt19937 rng_;
  V3 pos_, vel_;
  bool have_odom_{false};
  double energy_{0}, last_t_{-1};
  GroundTruth::SharedPtr gt_;
  std::map<std::string, Peer> peers_;
  std::vector<LocalTrack> tracks_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<DetectionArray>::SharedPtr det_pub_;
  rclcpp::Publisher<swarm_interfaces::msg::LinkStatus>::SharedPtr link_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub_;
};

}  // namespace swarm_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swarm_control::SensorSim>());
  rclcpp::shutdown();
  return 0;
}
