#include "jev_ai_bt/nodes.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace BT
{

template<>
jev_ai::State convertFromString<jev_ai::State>(StringView str)
{
  jev_ai::State out;
  for (auto part : splitString(str, ';')) {
    const std::string kv(part);
    const auto eq = kv.find('=');
    if (eq == std::string::npos) {
      throw RuntimeError("expected key=value, got: ", kv);
    }
    out[kv.substr(0, eq)] = kv.substr(eq + 1);
  }
  return out;
}

template<>
jev_ai::ClassSets convertFromString<jev_ai::ClassSets>(StringView str)
{
  jev_ai::ClassSets out;
  for (const auto & [question, classes] : convertFromString<jev_ai::State>(str)) {
    for (auto c : splitString(classes, ',')) {
      out[question].insert(std::string(c));
    }
  }
  return out;
}

}  // namespace BT

namespace jev_ai_bt
{

namespace
{

using BT::NodeStatus;
using Clock = std::chrono::steady_clock;

rclcpp::Node * ros_node(const BT::TreeNode & n)
{
  return n.config().blackboard->get<rclcpp::Node::SharedPtr>("node").get();
}

// Results land here from ROS callbacks; a halted node bumps `gen` so late answers are dropped.
// Shared with the callbacks so they stay valid even if the tree is destroyed first.
template<class T>
struct Inbox
{
  std::mutex m;
  uint64_t gen = 0;
  std::optional<T> value;
  std::string error;
  bool done = false;

  uint64_t reset()
  {
    std::lock_guard<std::mutex> lock(m);
    value.reset();
    error.clear();
    done = false;
    return ++gen;
  }
  void put(uint64_t g, std::optional<T> v, const std::string & e)
  {
    std::lock_guard<std::mutex> lock(m);
    if (g == gen) {
      value = std::move(v);
      error = e;
      done = true;
    }
  }
};

// --- Decide: one decision from decide_server (the Python engine) -------------------------------

class Decide : public BT::StatefulActionNode
{
public:
  Decide(const std::string & name, const BT::NodeConfig & config)
  : StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("question_set"),
      BT::InputPort<jev_ai::State>("state", "key=value;..."),
      BT::InputPort<jev_ai::ClassSets>("mask", "forbidden: question=class,class;..."),
      BT::InputPort<jev_ai::Classes>("require", "forced: question=class;..."),
      BT::InputPort<std::string>("trigger", "", "app event name ('' = a tick)"),
      BT::InputPort<std::string>("agent", "", "engine key ('' = the ROS node's name)"),
      BT::InputPort<std::string>("question", "", "question for `selected` ('' = the first)"),
      BT::InputPort<bool>("wait_final", false, "false: succeed on the first (PROVISIONAL) "
        "decision, act-then-revise; true: wait for the final phase"),
      BT::InputPort<int>("timeout_ms", 15000, "fail if no decision by then"),
      BT::OutputPort<std::string>("selected"),
      BT::OutputPort<std::string>("phase"),
      BT::OutputPort<jev_ai::Decision>("decision")};
  }

  NodeStatus onStart() override
  {
    if (!client_) {
      auto qs = getInput<std::string>("question_set");
      if (!qs) {
        throw BT::RuntimeError(name(), ": question_set is required");
      }
      client_ = std::make_unique<jev_ai::DecisionClient>(
        ros_node(*this), *qs, getInput<std::string>("agent").value_or(""));
      client_->on_decision([inbox = inbox_, this](const jev_ai::Decision & d) {
          inbox->put(sent_, d, "");
        });
    }
    auto state = getInput<jev_ai::State>("state");
    if (!state) {
      RCLCPP_ERROR(ros_node(*this)->get_logger(), "%s: %s", name().c_str(),
        state.error().c_str());
      return NodeStatus::FAILURE;
    }
    sent_ = inbox_->reset();
    started_ = Clock::now();
    const auto trigger = getInput<std::string>("trigger").value_or("");
    const bool ok = client_->request(
      *state, getInput<jev_ai::ClassSets>("mask").value_or(jev_ai::ClassSets{}),
      getInput<jev_ai::Classes>("require").value_or(jev_ai::Classes{}),
      trigger.empty() ? std::vector<std::string>{} : std::vector<std::string>{trigger});
    return ok ? NodeStatus::RUNNING : NodeStatus::FAILURE;
  }

  NodeStatus onRunning() override
  {
    std::optional<jev_ai::Decision> d;
    {
      std::lock_guard<std::mutex> lock(inbox_->m);
      d = inbox_->value;
    }
    const auto waited = Clock::now() - started_;
    if (!d || (getInput<bool>("wait_final").value_or(false) && !d->final)) {
      const bool failed = !d && !client_->busy();          // rejected / aborted
      const bool late = waited > std::chrono::milliseconds(
        getInput<int>("timeout_ms").value_or(15000));
      return failed || late ? NodeStatus::FAILURE : NodeStatus::RUNNING;
    }
    auto question = getInput<std::string>("question").value_or("");
    if (question.empty() && !d->msg.selected.empty()) {
      question = d->msg.selected.front().question;
    }
    setOutput("selected", d->selected(question));
    setOutput("phase", d->phase());
    setOutput("decision", *d);
    return NodeStatus::SUCCESS;
  }

  void onHalted() override {inbox_->reset();}

private:
  std::unique_ptr<jev_ai::DecisionClient> client_;
  std::shared_ptr<Inbox<jev_ai::Decision>> inbox_ = std::make_shared<Inbox<jev_ai::Decision>>();
  std::atomic<uint64_t> sent_{0};
  Clock::time_point started_;
};

// --- JevAsk: one raw System 1 question (jev/decide) ---------------------------------------------

class JevAsk : public BT::StatefulActionNode
{
public:
  JevAsk(const std::string & name, const BT::NodeConfig & config)
  : StatefulActionNode(name, config), jev_(ros_node(*this)) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<jev_ai::State>("state", "key=value;..."),
      BT::InputPort<std::string>("question", "ask", "question name"),
      BT::InputPort<std::string>("type", "choice", "choice | score | noul"),
      BT::InputPort<std::string>("instructions"),
      BT::InputPort<std::string>("classes", "", "a;b;c (noul: true;false)"),
      BT::InputPort<std::string>("criteria", "", "aligned with classes, ';'-separated"),
      BT::InputPort<int>("timeout_ms", 1000, "deadline"),
      BT::OutputPort<std::string>("answer", "argmax class"),
      BT::OutputPort<std::vector<double>>("probabilities"),
      BT::OutputPort<double>("confidence")};
  }

  NodeStatus onStart() override
  {
    auto state = getInput<jev_ai::State>("state");
    auto instructions = getInput<std::string>("instructions");
    if (!state || !instructions) {
      RCLCPP_ERROR(ros_node(*this)->get_logger(), "%s: state and instructions are required",
        name().c_str());
      return NodeStatus::FAILURE;
    }
    jev_ai_msgs::msg::Question q;
    q.name = getInput<std::string>("question").value_or("ask");
    const auto type = getInput<std::string>("type").value_or("choice");
    q.type = type == "noul" ? jev_ai_msgs::msg::Question::NOUL :
      type == "score" ? jev_ai_msgs::msg::Question::SCORE : jev_ai_msgs::msg::Question::CHOICE;
    q.instructions = *instructions;
    for (auto c : BT::splitString(getInput<std::string>("classes").value_or(""), ';')) {
      q.classes.emplace_back(c);
    }
    if (q.classes.empty() && q.type == jev_ai_msgs::msg::Question::NOUL) {
      q.classes = {"true", "false"};
    }
    for (auto c : BT::splitString(getInput<std::string>("criteria").value_or(""), ';')) {
      q.criteria.emplace_back(c);
    }
    const auto gen = inbox_->reset();
    jev_.ask(
      *state, {q}, [inbox = inbox_, gen](auto res, const std::string & error) {
        std::string err = error;
        if (res && (res->answers.empty() || !res->question_errors.front().empty())) {
          err = res->question_errors.empty() ? "no answer" : res->question_errors.front();
        }
        inbox->put(gen, err.empty() ? std::optional<jev_ai_msgs::msg::Distribution>(
          res->answers.front()) : std::nullopt, err);
      }, std::chrono::milliseconds(getInput<int>("timeout_ms").value_or(1000)));
    return NodeStatus::RUNNING;
  }

  NodeStatus onRunning() override
  {
    std::lock_guard<std::mutex> lock(inbox_->m);
    if (!inbox_->done) {
      return NodeStatus::RUNNING;
    }
    if (!inbox_->value) {
      RCLCPP_WARN(ros_node(*this)->get_logger(), "%s: %s", name().c_str(),
        inbox_->error.c_str());
      return NodeStatus::FAILURE;
    }
    const auto & d = *inbox_->value;
    const auto best = std::max_element(d.probabilities.begin(), d.probabilities.end());
    setOutput("answer", d.classes[best - d.probabilities.begin()]);
    setOutput("probabilities", std::vector<double>(d.probabilities.begin(),
      d.probabilities.end()));
    setOutput("confidence", d.confidence);
    return NodeStatus::SUCCESS;
  }

  void onHalted() override {inbox_->reset();}

private:
  jev_ai::JevClient jev_;
  std::shared_ptr<Inbox<jev_ai_msgs::msg::Distribution>> inbox_ =
    std::make_shared<Inbox<jev_ai_msgs::msg::Distribution>>();
};

// --- Reason: one System 2 call (llm/reason) -----------------------------------------------------

class Reason : public BT::StatefulActionNode
{
public:
  using Action = jev_ai_msgs::action::Reason;

  Reason(const std::string & name, const BT::NodeConfig & config)
  : StatefulActionNode(name, config), llm_(ros_node(*this)) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("system"),
      BT::InputPort<std::string>("context"),
      BT::InputPort<std::string>("response_schema", "", "JSON Schema ('' = free text)"),
      BT::InputPort<std::string>("model", "", "'' = the server default"),
      BT::InputPort<int>("timeout_ms", 6500, "deadline; the goal is cancelled after it"),
      BT::OutputPort<std::string>("text"),
      BT::OutputPort<std::string>("json")};
  }

  NodeStatus onStart() override
  {
    Action::Goal goal;
    goal.system = getInput<std::string>("system").value_or("");
    goal.context = getInput<std::string>("context").value_or("");
    goal.response_schema = getInput<std::string>("response_schema").value_or("");
    goal.model = getInput<std::string>("model").value_or("");
    const auto gen = inbox_->reset();
    llm_.reason(
      goal, [inbox = inbox_, gen](const Action::Result & r) {
        inbox->put(gen, r.success ? std::optional<Action::Result>(r) : std::nullopt, r.error);
      }, std::chrono::milliseconds(getInput<int>("timeout_ms").value_or(6500)));
    return NodeStatus::RUNNING;
  }

  NodeStatus onRunning() override
  {
    std::lock_guard<std::mutex> lock(inbox_->m);
    if (!inbox_->done) {
      return NodeStatus::RUNNING;
    }
    if (!inbox_->value) {
      RCLCPP_WARN(ros_node(*this)->get_logger(), "%s: %s", name().c_str(),
        inbox_->error.c_str());
      return NodeStatus::FAILURE;
    }
    setOutput("text", inbox_->value->text);
    setOutput("json", inbox_->value->json);
    return NodeStatus::SUCCESS;
  }

  void onHalted() override {inbox_->reset();}

private:
  jev_ai::ReasonClient llm_;
  std::shared_ptr<Inbox<Action::Result>> inbox_ = std::make_shared<Inbox<Action::Result>>();
};

}  // namespace

void register_nodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<Decide>("Decide");
  factory.registerNodeType<JevAsk>("JevAsk");
  factory.registerNodeType<Reason>("Reason");
}

}  // namespace jev_ai_bt

BT_REGISTER_NODES(factory)
{
  jev_ai_bt::register_nodes(factory);
}
