// comms_sim (B2): the radio between the drones and the coordinator (SYSTEM_MODEL §4.8).
// Relays jev_ai hierarchy traffic: /uavK/reports -> /coordinator/reports and
// /coordinator/directives -> /uavK/directives, dropping each message with probability 1 - q
// (the drone's /uavK/link quality) and delaying it by latency_ms + U(0, latency_jitter_ms).
// Delivery counts go to /diagnostics (shown on the jev_ai dashboard).
// ponytail: ROS actions (Task) bypass this; the coordinator refuses to dispatch at q < 0.2.
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <jev_ai_msgs/msg/directive.hpp>
#include <jev_ai_msgs/msg/report.hpp>
#include <rclcpp/rclcpp.hpp>

#include "swarm_control/world.hpp"
#include "swarm_interfaces/msg/link_status.hpp"

namespace swarm_control
{

using jev_ai_msgs::msg::Directive;
using jev_ai_msgs::msg::Report;

class CommsSim : public rclcpp::Node
{
public:
  CommsSim()
  : Node("comms_sim")
  {
    p_ = load_params(*this);
    rng_.seed(static_cast<unsigned>(p_.seed * 7919 + 1));
    // the jev_ai Hierarchy QoS: reports best effort, directives reliable
    const auto report_qos = rclcpp::QoS(50).best_effort();
    const auto directive_qos = rclcpp::QoS(50).reliable();
    up_ = create_publisher<Report>("/coordinator/reports", report_qos);
    for (int k = 1; k <= p_.num_drones; ++k) {
      const std::string d = "uav" + std::to_string(k);
      q_[d] = 1.0;
      subs_.push_back(create_subscription<swarm_interfaces::msg::LinkStatus>(
          "/" + d + "/link", 10,
          [this, d](swarm_interfaces::msg::LinkStatus::SharedPtr m) {q_[d] = m->quality;}));
      subs_.push_back(create_subscription<Report>("/" + d + "/reports", report_qos,
        [this, d](Report::SharedPtr m) {
          relay(d, "up", [this, m] {up_->publish(*m);});
        }));
      down_[d] = create_publisher<Directive>("/" + d + "/directives", directive_qos);
    }
    subs_.push_back(create_subscription<Directive>("/coordinator/directives", directive_qos,
      [this](Directive::SharedPtr m) {
        for (auto & [d, pub] : down_) {
          if (m->target == "*" || m->target == d || m->target == "/" + d) {
            auto p = pub;
            relay(d, "down", [p, m] {p->publish(*m);});
          }
        }
      }));
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    timers_.push_back(create_wall_timer(std::chrono::milliseconds(20), [this] {deliver();}));
    timers_.push_back(create_wall_timer(std::chrono::seconds(1), [this] {diagnose();}));
  }

private:
  double now_s() {return get_clock()->now().seconds();}

  void relay(const std::string & drone, const std::string & dir, std::function<void()> send)
  {
    std::uniform_real_distribution<double> U(0.0, 1.0);
    auto & c = counts_[drone + " " + dir];
    if (U(rng_) >= q_[drone]) {
      c.second++;
      return;
    }
    c.first++;
    const double delay = (p_.latency_ms + U(rng_) * p_.latency_jitter_ms) / 1000.0;
    queue_.push_back({now_s() + delay, std::move(send)});
  }

  void deliver()
  {
    const double t = now_s();
    for (auto it = queue_.begin(); it != queue_.end(); ) {
      if (it->first <= t) {it->second(); it = queue_.erase(it);} else {++it;}
    }
  }

  void diagnose()
  {
    diagnostic_msgs::msg::DiagnosticArray a;
    a.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "comms_sim";
    s.hardware_id = "swarm";
    int sent = 0, lost = 0;
    for (const auto & [k, c] : counts_) {sent += c.first; lost += c.second;}
    s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    s.message = std::to_string(sent) + " relayed, " + std::to_string(lost) + " dropped";
    for (const auto & [d, q] : q_) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = d + " link";
      char buf[64];
      const auto & up = counts_[d + " up"];
      const auto & dn = counts_[d + " down"];
      snprintf(buf, sizeof(buf), "q %.2f; up %d/%d, down %d/%d", q, up.first, up.first + up.second,
        dn.first, dn.first + dn.second);
      kv.value = buf;
      s.values.push_back(kv);
    }
    a.status.push_back(s);
    diag_pub_->publish(a);
  }

  Params p_;
  std::mt19937 rng_;
  std::map<std::string, double> q_;
  std::map<std::string, std::pair<int, int>> counts_;   // delivered, dropped
  std::deque<std::pair<double, std::function<void()>>> queue_;
  rclcpp::Publisher<Report>::SharedPtr up_;
  std::map<std::string, rclcpp::Publisher<Directive>::SharedPtr> down_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
};

}  // namespace swarm_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swarm_control::CommsSim>());
  rclcpp::shutdown();
  return 0;
}
