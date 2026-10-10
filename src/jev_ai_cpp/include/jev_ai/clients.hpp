// jev_ai C++ clients (FRAMEWORK §2, D2). C++ owns no decision maths: DecisionClient reaches the
// one Python engine through decide_server, so decisions are identical to Python's.
//
//   jev_ai::DecisionClient decide(this, "next_step");
//   decide.on_decision([&](const jev_ai::Decision & d) { act(d.selected("next_action")); });
//   decide.start(1.0, encode, mask);             // or decide.request(state, mask)
//
// JevClient (jev/decide) and ReasonClient (llm/reason) are raw async clients with deadlines,
// for apps that want System 1 or System 2 directly.
#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "jev_ai_msgs/action/decide.hpp"
#include "jev_ai_msgs/action/reason.hpp"
#include "jev_ai_msgs/srv/jev_decide.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace jev_ai
{

using State = std::map<std::string, std::string>;               // state key -> interpreted text
using ClassSets = std::map<std::string, std::set<std::string>>;  // question -> classes (mask)
using Classes = std::map<std::string, std::string>;             // question -> class (require)

/// One decision. PROVISIONAL and REVISED arrive as feedback (final() false), then the final
/// phase: COMMITTED | KEPT | FALLBACK | FORCED. Act on every one you receive.
struct Decision
{
  std::string question_set;
  jev_ai_msgs::msg::Decision msg;
  bool final = false;

  const std::string & phase() const {return msg.phase;}
  std::string selected(const std::string & question) const;     // "" if none
  double value(const std::string & key, double fallback = NAN) const;   // e.g. urgency.level_mean
};

class DecisionClient
{
public:
  using Callback = std::function<void (const Decision &)>;

  /// `agent` defaults to the node's fully qualified name (the engine instance key).
  DecisionClient(
    rclcpp::Node * node, std::string question_set, std::string agent = "",
    const std::string & action = "decide");

  void on_decision(Callback cb) {cb_ = std::move(cb);}

  /// Send one decision request. A request while one is in flight preempts it (newest state
  /// wins). False if decide_server is not available.
  bool request(
    const State & state, const ClassSets & mask = {}, const Classes & require = {},
    const std::vector<std::string> & triggers = {});

  /// Convenience loop: at `hz`, when nothing is in flight, encode and request.
  void start(
    double hz, std::function<State()> encode, std::function<ClassSets()> mask = nullptr,
    std::function<Classes()> require = nullptr);
  void stop() {timer_.reset();}

  /// App event (§5.1 trigger). With start(): requests now; otherwise sent with the next request.
  void trigger(const std::string & name);
  /// §5.1 completed() / commit_point(), sent with the next request.
  void completed(const std::string & question) {queue("completed:" + question, false);}
  void commit_point(const std::string & question) {queue("commit_point:" + question, false);}

  bool busy() const {return busy_;}

private:
  using Decide = jev_ai_msgs::action::Decide;
  void queue(const std::string & trigger, bool now);
  void loop_once();

  rclcpp::Node * node_;
  std::string question_set_, agent_;
  rclcpp_action::Client<Decide>::SharedPtr client_;
  Callback cb_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::function<State()> encode_;
  std::function<ClassSets()> mask_;
  std::function<Classes()> require_;
  std::mutex mutex_;
  std::vector<std::string> triggers_;
  std::atomic<uint64_t> seq_{0}, last_final_id_{0};
  std::atomic<bool> busy_{false};
};

class JevClient
{
public:
  using Response = jev_ai_msgs::srv::JevDecide::Response;
  /// `response` is null on failure (unavailable, deadline, or success == false: see `error`).
  using Callback = std::function<void (Response::SharedPtr response, const std::string & error)>;

  explicit JevClient(rclcpp::Node * node, const std::string & service = "jev/decide");
  void ask(
    const State & state, const std::vector<jev_ai_msgs::msg::Question> & questions, Callback cb,
    std::chrono::milliseconds deadline = std::chrono::milliseconds(1000),
    const std::string & model = "");

private:
  rclcpp::Node * node_;
  rclcpp::Client<jev_ai_msgs::srv::JevDecide>::SharedPtr client_;
};

class ReasonClient
{
public:
  using Reason = jev_ai_msgs::action::Reason;
  /// Always called once; `result.success` false with `result.error` on any failure.
  using Callback = std::function<void (const Reason::Result & result)>;

  explicit ReasonClient(rclcpp::Node * node, const std::string & action = "llm/reason");
  /// The goal is cancelled (aborting the HTTP stream server-side) when the deadline passes.
  void reason(
    const Reason::Goal & goal, Callback cb,
    std::chrono::milliseconds deadline = std::chrono::milliseconds(6500));

private:
  rclcpp::Node * node_;
  rclcpp_action::Client<Reason>::SharedPtr client_;
};

}  // namespace jev_ai
