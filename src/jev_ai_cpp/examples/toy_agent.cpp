// C++ toy_patrol agent (A6): the Python toy_patrol world, deciding through decide_server.
// A component (jev_ai_cpp::ToyAgent) and an executable (toy_agent).
//
//   ros2 launch jev_ai_examples toy_patrol.launch.py cpp_agents:=2
#include <random>
#include <string>

#include "jev_ai/clients.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace jev_ai_cpp
{

using namespace std::chrono_literals;

class ToyAgent : public rclcpp::Node
{
public:
  explicit ToyAgent(const rclcpp::NodeOptions & options)
  : Node("toy_agent", options), decide_(this, "next_step"),
    rng_(static_cast<unsigned>(declare_parameter("seed", 0)))
  {
    energy_ = std::uniform_real_distribution<double>(60, 95)(rng_);
    decide_.on_decision([this](const jev_ai::Decision & d) {act(d);});
    decide_.start(
      1.0, [this] {return encode();},
      [this] {return detection_ == "none" ? jev_ai::ClassSets{{"next_action", {"inspect"}}} :
      jev_ai::ClassSets{};},
      [this] {return energy_ <= 5 && task_ != "halted" ?
      jev_ai::Classes{{"next_action", "halt_now"}} : jev_ai::Classes{};});
    timer_ = create_wall_timer(500ms, [this] {step();});
  }

private:
  jev_ai::State encode() const
  {
    const int e = static_cast<int>(energy_ + 0.5);
    const char * level = e >= 40 ? "ok" : e >= 15 ? "low" : "critical";
    return {{"task", task_}, {"energy", std::string(level) + " (" + std::to_string(e) + "%)"},
      {"link", link_}, {"hazards", hazard_}, {"detection", detection_},
      {"orders", "none"}};   // ponytail: C++ agents don't take supervisor directives (A7: Python)
  }

  void act(const jev_ai::Decision & d)
  {
    const auto action = d.selected("next_action");
    const double now = this->now().seconds();
    if (action == "halt_now" && task_ != "halted") {
      task_ = "halted";
      inspect_done_ = 0;
      reset_at_ = now + 8.0;
    } else if (action == "inspect" && inspect_done_ == 0 && task_ != "halted") {
      task_ = "inspecting " + detection_.substr(0, detection_.find(" at "));
      commit_at_ = now + 1.0;
      inspect_done_ = now + 4.0;
    } else if (action == "continue" && task_ != "halted") {
      task_ = "patrolling";
      inspect_done_ = 0;
    }
    RCLCPP_INFO(
      get_logger(), "%s #%lu: next_action=%s urgency=%.2f", d.phase().c_str(),
      static_cast<unsigned long>(d.msg.decision_id), action.c_str(),
      d.value("urgency.level_mean", 0.0));
  }

  void step()
  {
    const double now = this->now().seconds();
    auto r = [this] {return std::uniform_real_distribution<double>(0, 1)(rng_);};
    if (task_ == "halted") {
      if (now >= reset_at_) {
        energy_ = 100;
        task_ = "patrolling";
        decide_.trigger("battery_swapped");
      }
      return;
    }
    energy_ = std::max(0.0, energy_ - (inspect_done_ ? 0.45 : 0.25));
    if (inspect_done_) {
      if (commit_at_ && now >= commit_at_) {
        decide_.commit_point("next_action");
        commit_at_ = 0;
      }
      if (now >= inspect_done_) {
        inspect_done_ = 0;
        detection_ = "none";
        task_ = "patrolling";
        decide_.completed("next_action");
      }
    }
    if (detection_ == "none" && r() < 0.03) {
      detection_ = std::string(r() < 0.4 ? "person-like shape" : "debris") + " at " +
        std::to_string(5 + static_cast<int>(r() * 35)) + " m";
      decide_.trigger("detection");
    } else if (detection_ != "none" && !inspect_done_ && r() < 0.02) {
      detection_ = "none";
    }
    if (hazard_ == "none" && r() < 0.01) {
      hazard_ = "smoke";
      decide_.trigger("hazard");
    } else if (hazard_ != "none" && r() < 0.05) {
      hazard_ = "none";
    }
    link_ = r() < 0.1 ? "weak" : "good";
  }

  jev_ai::DecisionClient decide_;
  std::mt19937 rng_;
  rclcpp::TimerBase::SharedPtr timer_;
  double energy_ = 80, reset_at_ = 0, commit_at_ = 0, inspect_done_ = 0;
  std::string task_ = "patrolling", detection_ = "none", hazard_ = "none", link_ = "good";
};

}  // namespace jev_ai_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(jev_ai_cpp::ToyAgent)
