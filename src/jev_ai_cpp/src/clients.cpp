#include "jev_ai/clients.hpp"

#include <utility>

namespace jev_ai
{

namespace
{

using namespace std::chrono_literals;

// One call with a deadline: whichever of {answer, deadline} claims first reports; the other is
// ignored. The timer's callback owns this object, so claim() drops the timer to free both.
struct Call
{
  std::atomic<bool> done{false};
  rclcpp::TimerBase::SharedPtr timer;
  bool claim()
  {
    if (done.exchange(true)) {
      return false;
    }
    if (timer) {
      timer->cancel();
      timer.reset();
    }
    return true;
  }
};

jev_ai_msgs::msg::ClassRef ref(const std::string & q, const std::string & c)
{
  jev_ai_msgs::msg::ClassRef r;
  r.question = q;
  r.cls = c;
  return r;
}

std::vector<diagnostic_msgs::msg::KeyValue> key_values(const State & state)
{
  std::vector<diagnostic_msgs::msg::KeyValue> out;
  for (const auto & [k, v] : state) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = k;
    kv.value = v;
    out.push_back(kv);
  }
  return out;
}

}  // namespace

// --- Decision ----------------------------------------------------------------------------------

std::string Decision::selected(const std::string & question) const
{
  for (const auto & r : msg.selected) {
    if (r.question == question) {
      return r.cls;
    }
  }
  return "";
}

double Decision::value(const std::string & key, double fallback) const
{
  for (const auto & kv : msg.values) {
    if (kv.key == key) {
      try {
        return std::stod(kv.value);
      } catch (const std::exception &) {
        return fallback;
      }
    }
  }
  return fallback;
}

// --- DecisionClient ----------------------------------------------------------------------------

DecisionClient::DecisionClient(
  rclcpp::Node * node, std::string question_set, std::string agent, const std::string & action)
: node_(node), question_set_(std::move(question_set)),
  agent_(agent.empty() ? node->get_fully_qualified_name() : std::move(agent)),
  client_(rclcpp_action::create_client<Decide>(node, action))
{
}

bool DecisionClient::request(
  const State & state, const ClassSets & mask, const Classes & require,
  const std::vector<std::string> & triggers)
{
  if (!client_->action_server_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "%s: decide_server unavailable",
      question_set_.c_str());
    return false;
  }
  Decide::Goal goal;
  goal.agent = agent_;
  goal.question_set = question_set_;
  goal.state = key_values(state);
  for (const auto & [q, classes] : mask) {
    for (const auto & c : classes) {
      goal.mask.push_back(ref(q, c));
    }
  }
  for (const auto & [q, c] : require) {
    goal.require.push_back(ref(q, c));
  }
  goal.triggers = triggers;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal.triggers.insert(goal.triggers.end(), triggers_.begin(), triggers_.end());
    triggers_.clear();
  }
  const uint64_t seq = ++seq_;
  busy_ = true;
  rclcpp_action::Client<Decide>::SendGoalOptions opt;
  opt.goal_response_callback = [this, seq](auto handle) {
      if (!handle && seq == seq_) {
        busy_ = false;
        RCLCPP_WARN(node_->get_logger(), "%s: decide goal rejected", question_set_.c_str());
      }
    };
  opt.feedback_callback = [this, seq](auto, const std::shared_ptr<const Decide::Feedback> fb) {
      if (seq == seq_) {
        deliver(Decision{question_set_, fb->decision, false});
      }
    };
  opt.result_callback = [this, seq](const auto & wrapped) {
      if (seq != seq_) {
        return;                                    // preempted by a newer request
      }
      if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
        deliver(Decision{question_set_, wrapped.result->decision, true});
      } else {
        RCLCPP_WARN(
          node_->get_logger(), "%s: decision failed %s", question_set_.c_str(),
          wrapped.result ? wrapped.result->decision.rationale.c_str() : "");
      }
      busy_ = false;                               // after delivery: !busy && nothing = failed
    };
  client_->async_send_goal(goal, opt);
  return true;
}

void DecisionClient::deliver(Decision d)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::pair<uint64_t, std::string> key{d.msg.decision_id, d.msg.phase};
    d.standing = key == last_;
    last_ = key;
  }
  if (cb_) {
    cb_(d);
  }
}

void DecisionClient::start(
  double hz, std::function<State()> encode, std::function<ClassSets()> mask,
  std::function<Classes()> require)
{
  encode_ = std::move(encode);
  mask_ = std::move(mask);
  require_ = std::move(require);
  timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / hz), [this]() {
      if (!busy_) {
        loop_once();
      }
    });
}

void DecisionClient::loop_once()
{
  request(encode_(), mask_ ? mask_() : ClassSets{}, require_ ? require_() : Classes{});
}

void DecisionClient::trigger(const std::string & name)
{
  queue(name, true);
}

void DecisionClient::queue(const std::string & trigger, bool now)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    triggers_.push_back(trigger);
  }
  if (now && timer_) {
    loop_once();                                   // preempts what is in flight
  }
}

// --- JevClient ---------------------------------------------------------------------------------

JevClient::JevClient(rclcpp::Node * node, const std::string & service)
: node_(node), client_(node->create_client<jev_ai_msgs::srv::JevDecide>(service))
{
}

void JevClient::ask(
  const State & state, const std::vector<jev_ai_msgs::msg::Question> & questions, Callback cb,
  std::chrono::milliseconds deadline, const std::string & model)
{
  if (!client_->service_is_ready()) {
    return cb(nullptr, "jev/decide unavailable");
  }
  auto req = std::make_shared<jev_ai_msgs::srv::JevDecide::Request>();
  req->state = key_values(state);
  req->questions = questions;
  req->model = model;
  auto call = std::make_shared<Call>();
  auto id = std::make_shared<std::atomic<int64_t>>(-1);
  auto client = client_;
  call->timer = node_->create_wall_timer(
    deadline, [call, id, client, cb]() {
      if (call->claim()) {
        client->remove_pending_request(*id);
        cb(nullptr, "deadline");
      }
    });
  *id = client_->async_send_request(
    req, [call, cb](rclcpp::Client<jev_ai_msgs::srv::JevDecide>::SharedFuture f) {
      if (!call->claim()) {
        return;
      }
      auto res = f.get();
      if (res->success) {
        cb(res, "");
      } else {
        cb(nullptr, res->error);
      }
    }).request_id;
}

// --- ReasonClient ------------------------------------------------------------------------------

ReasonClient::ReasonClient(rclcpp::Node * node, const std::string & action)
: node_(node), client_(rclcpp_action::create_client<Reason>(node, action))
{
}

void ReasonClient::reason(const Reason::Goal & goal, Callback cb, std::chrono::milliseconds deadline)
{
  auto fail = [cb](const std::string & error) {
      Reason::Result r;
      r.success = false;
      r.error = error;
      cb(r);
    };
  if (!client_->action_server_is_ready()) {
    return fail("llm/reason unavailable");
  }
  struct ReasonCall : Call
  {
    std::mutex m;
    rclcpp_action::ClientGoalHandle<Reason>::SharedPtr handle;
  };
  auto call = std::make_shared<ReasonCall>();
  auto client = client_;
  call->timer = node_->create_wall_timer(
    deadline, [call, client, fail]() {
      if (call->claim()) {
        std::lock_guard<std::mutex> lock(call->m);
        if (call->handle) {
          client->async_cancel_goal(call->handle);   // aborts the HTTP stream server-side
        }
        fail("deadline");
      }
    });
  rclcpp_action::Client<Reason>::SendGoalOptions opt;
  opt.goal_response_callback = [call, client, fail](auto handle) {
      if (!handle) {
        if (call->claim()) {
          fail("goal rejected");
        }
        return;
      }
      std::lock_guard<std::mutex> lock(call->m);
      call->handle = handle;
      if (call->done) {                              // deadline passed before acceptance
        client->async_cancel_goal(handle);
      }
    };
  opt.result_callback = [call, cb, fail](const auto & wrapped) {
      if (!call->claim()) {
        return;
      }
      if (wrapped.result) {
        cb(*wrapped.result);
      } else {
        fail("no result");
      }
    };
  client_->async_send_goal(goal, opt);
}

}  // namespace jev_ai
