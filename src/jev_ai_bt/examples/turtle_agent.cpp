// turtlesim example (A9), C++ variants: the Python turtle (jev_ai_examples turtle_agent) ported,
// deciding through decide_server either with DecisionClient (mode:=client) or with a behaviour
// tree (mode:=bt, bt/turtle.xml). Same catalog, same masks, same decisions as the Python turtle.
#include <cmath>
#include <csignal>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <set>
#include <thread>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "jev_ai_bt/nodes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

namespace
{

constexpr double kArena = 11.088889, kWallMask = 1.2, kReached = 0.5, kTurnS = 1.0,
  kTurnRate = 1.6;

double wall_distance(double x, double y, double th)
{
  const double c = std::cos(th), s = std::sin(th);
  double best = kArena;
  if (c > 1e-6) {best = std::min(best, (kArena - x) / c);}
  if (c < -1e-6) {best = std::min(best, -x / c);}
  if (s > 1e-6) {best = std::min(best, (kArena - y) / s);}
  if (s < -1e-6) {best = std::min(best, -y / s);}
  return std::max(0.0, best);
}

// (distance, bearing; + = left)
std::pair<double, double> relative(const turtlesim::msg::Pose & p, double tx, double ty)
{
  const double b = std::atan2(ty - p.y, tx - p.x) - p.theta;
  return {std::hypot(tx - p.x, ty - p.y), std::atan2(std::sin(b), std::cos(b))};
}

std::string fmt(const char * f, double a, double b = 0, double c = 0)
{
  char buf[96];
  std::snprintf(buf, sizeof(buf), f, a, b, c);
  return buf;
}

std::string side(double b)
{
  return fmt("%.0f° ", std::abs(b) * 180 / M_PI) + (b > 0 ? "left" : "right");
}

class Turtle
{
public:
  explicit Turtle(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    name_ = node->declare_parameter("turtle", "turtle2");
    for (const auto & t : node->declare_parameter("turtles", std::vector<std::string>{})) {
      if (t != name_) {
        subs_.push_back(node->create_subscription<turtlesim::msg::Pose>(
          "/" + t + "/pose", 10, [this, t](turtlesim::msg::Pose::SharedPtr m) {
            std::lock_guard<std::mutex> lock(m_);
            others_[t] = *m;
          }));
      }
    }
    subs_.push_back(node->create_subscription<turtlesim::msg::Pose>(
      "/" + name_ + "/pose", 10, [this](turtlesim::msg::Pose::SharedPtr m) {
        std::lock_guard<std::mutex> lock(m_);
        pose_ = *m;
        have_pose_ = true;
      }));
    alert_sub_ = node->create_subscription<geometry_msgs::msg::Point>(
      "/alerts", 10, [this](geometry_msgs::msg::Point::SharedPtr m) {on_alert(*m);});
    cmd_ = node->create_publisher<geometry_msgs::msg::Twist>("/" + name_ + "/cmd_vel", 10);
    alerts_ = node->create_publisher<geometry_msgs::msg::Point>("/alerts", 10);
  }

  const std::string & name() const {return name_;}

  jev_ai::State encode()
  {
    std::lock_guard<std::mutex> lock(m_);
    turtlesim::msg::Pose p = pose_;
    if (!have_pose_) {
      p.x = p.y = kArena / 2;
    }
    const double wall = wall_distance(p.x, p.y, p.theta);
    std::string alert = "none", neighbour = "none";
    if (has_alert_) {
      auto [d, b] = relative(p, ax_, ay_);
      std::string nearer;
      for (const auto & [t, q] : others_) {
        if (std::hypot(q.x - ax_, q.y - ay_) < d) {nearer = t;}
      }
      alert = fmt("at (%.1f, %.1f): %.1f m, ", ax_, ay_, d) + side(b) + "; " +
        (nearer.empty() ? "you are nearest" : nearer + " is nearer");
    }
    double best = 1e9;
    for (const auto & [t, q] : others_) {
      auto [d, b] = relative(p, q.x, q.y);
      if (d < best) {
        best = d;
        neighbour = t + fmt(" %.1f m, ", d) + side(b);
      }
    }
    return {{"wall_ahead", std::string(wall < 2 * kWallMask ? "close" : "clear") +
      fmt(" (%.1f m)", wall)}, {"position", fmt("(%.1f, %.1f)", p.x, p.y)},
      {"alert", alert}, {"neighbour", neighbour}};
  }

  jev_ai::ClassSets mask()
  {
    std::lock_guard<std::mutex> lock(m_);
    std::set<std::string> out;
    if (have_pose_ && wall_distance(pose_.x, pose_.y, pose_.theta) < kWallMask) {
      out.insert("forward");
    }
    if (!has_alert_) {
      out.insert("go_to_alert");
    }
    return {{"action", out}};
  }

  void act(const std::string & action)
  {
    std::lock_guard<std::mutex> lock(m_);
    if (action != action_) {
      RCLCPP_INFO(node_->get_logger(), "%s: %s", name_.c_str(), action.c_str());
    }
    action_ = action;
    since_ = node_->now().seconds();
  }

  /// 10 Hz. Returns true when the current action just completed (a decision event).
  bool drive()
  {
    std::lock_guard<std::mutex> lock(m_);
    geometry_msgs::msg::Twist tw;
    bool completed = false;
    const double elapsed = node_->now().seconds() - since_;
    if (have_pose_ && action_ == "forward") {
      tw.linear.x = wall_distance(pose_.x, pose_.y, pose_.theta) > 0.4 ? 1.5 : 0.0;
    } else if (action_ == "turn_left" || action_ == "turn_right") {
      tw.angular.z = action_ == "turn_left" ? kTurnRate : -kTurnRate;
      if (elapsed > kTurnS) {
        action_.clear();
        completed = true;
      }
    } else if (have_pose_ && action_ == "go_to_alert" && has_alert_) {
      auto [d, b] = relative(pose_, ax_, ay_);
      tw.angular.z = 4.0 * b;
      tw.linear.x = std::abs(b) < 0.5 ? std::min(1.5, d) : 0.0;
      if (d < kReached) {
        geometry_msgs::msg::Point clear;
        clear.x = ax_;
        clear.y = ay_;
        clear.z = 1.0;
        alerts_->publish(clear);
      }
    }
    cmd_->publish(tw);
    completed = completed || std::exchange(alert_done_, false);
    return completed;
  }

private:
  void on_alert(const geometry_msgs::msg::Point & m)
  {
    std::lock_guard<std::mutex> lock(m_);
    if (m.z == 0.0) {
      has_alert_ = true;
      ax_ = m.x;
      ay_ = m.y;
      new_alert_ = true;
    } else if (has_alert_ && std::hypot(m.x - ax_, m.y - ay_) < 1e-3) {
      has_alert_ = false;
      alert_done_ = action_ == "go_to_alert";
    }
  }

public:
  bool take_new_alert() {std::lock_guard<std::mutex> lock(m_); return std::exchange(new_alert_, false);}

private:
  rclcpp::Node::SharedPtr node_;
  std::string name_, action_;
  std::mutex m_;
  turtlesim::msg::Pose pose_;
  bool have_pose_ = false, has_alert_ = false, new_alert_ = false, alert_done_ = false;
  double ax_ = 0, ay_ = 0, since_ = 0;
  std::map<std::string, turtlesim::msg::Pose> others_;
  std::vector<rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr> subs_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr alert_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr alerts_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("turtle_agent");
  const auto mode = node->declare_parameter("mode", "client");
  Turtle turtle(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  if (mode == "client") {
    // DecisionClient: the start(hz) loop ticks; alerts and completions are decision events
    jev_ai::DecisionClient decide(node.get(), "move", "/" + turtle.name());
    decide.on_decision([&](const jev_ai::Decision & d) {
        if (!d.standing) {turtle.act(d.selected("action"));}
      });
    decide.start(2.0, [&] {return turtle.encode();}, [&] {return turtle.mask();});
    auto drive = node->create_wall_timer(
      std::chrono::milliseconds(100), [&]() {
        if (turtle.take_new_alert()) {decide.trigger("alert");}
        if (turtle.drive()) {decide.completed("action"); decide.trigger("completed");}
      });
    executor.spin();
  } else {
    // behaviour tree: bt/turtle.xml owns the decision loop; the app keeps the blackboard fresh
    // and applies each new decision ({decision}); events go in {event}, which the tree clears
    BT::BehaviorTreeFactory factory;
    jev_ai_bt::register_nodes(factory);
    auto bb = BT::Blackboard::create();
    bb->set("node", node);
    bb->set("agent", "/" + turtle.name());
    bb->set("event", std::string());
    bb->set("state", turtle.encode());
    bb->set("mask", turtle.mask());
    std::pair<uint64_t, std::string> applied;
    auto drive = node->create_wall_timer(
      std::chrono::milliseconds(100), [&]() {
        jev_ai::Decision d;
        if (bb->get("decision", d) && std::make_pair(d.msg.decision_id, d.phase()) != applied) {
          applied = {d.msg.decision_id, d.phase()};
          turtle.act(d.selected("action"));
        }
        const bool alert = turtle.take_new_alert(), done = turtle.drive();
        if (done) {
          bb->set("event", std::string("completed:action"));
        } else if (alert) {
          bb->set("event", std::string("alert"));
        }
        bb->set("state", turtle.encode());
        bb->set("mask", turtle.mask());
      });
    std::thread spinner([&]() {executor.spin();});
    auto tree = factory.createTreeFromFile(
      node->declare_parameter(
        "tree", ament_index_cpp::get_package_share_directory("jev_ai_bt") + "/bt/turtle.xml"),
      bb);
    while (rclcpp::ok()) {
      tree.tickOnce();
      tree.sleep(std::chrono::milliseconds(50));
    }
    executor.cancel();
    spinner.join();
  }
  std::signal(SIGINT, SIG_IGN);   // Ctrl-C arrives twice under launch (group + forward)
  rclcpp::shutdown();
  return 0;
}
