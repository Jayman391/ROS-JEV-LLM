// jev_ai BT toy (A8): a patrol robot whose decision loop is only XML (bt/toy_patrol.xml).
// The app part is the simulated world, which writes {state}/{mask}/{require}, and <Act>.
//
//   ros2 launch jev_ai_examples toy_patrol.launch.py agents:=0 bt_agents:=1
#include <csignal>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "jev_ai_bt/nodes.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

// The toy_patrol world, cut down: battery drains, detections come and go.
struct World
{
  std::mt19937 rng;
  double energy = 80;
  std::string task = "patrolling", detection = "none";
  std::string event;               // app trigger for the next Decide; Act consumes it
  double busy_until = 0;

  explicit World(unsigned seed)
  {
    std::seed_seq seq{seed};                     // small integer seeds mix poorly in mt19937
    rng.seed(seq);
  }

  double r() {return std::uniform_real_distribution<double>(0, 1)(rng);}

  void step(double now)
  {
    energy = std::max(0.0, energy - (task == "patrolling" ? 0.25 : 0.45));
    if (task != "patrolling" && now >= busy_until) {
      if (task == "halted") {
        energy = 100;
      }
      task = "patrolling";
      detection = "none";
    }
    if (detection == "none" && r() < 0.05) {
      detection = std::string(r() < 0.4 ? "person-like shape" : "debris") + " at " +
        std::to_string(5 + static_cast<int>(r() * 35)) + " m";
      event = "detection";
    } else if (detection != "none" && task == "patrolling" && r() < 0.04) {
      detection = "none";                        // passed it by
    }
  }

  jev_ai::State state() const
  {
    const int e = static_cast<int>(energy + 0.5);
    const char * level = e >= 40 ? "ok" : e >= 15 ? "low" : "critical";
    return {{"task", task}, {"energy", std::string(level) + " (" + std::to_string(e) + "%)"},
      {"link", "good"}, {"hazards", "none"}, {"detection", detection}, {"orders", "none"}};
  }
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_patrol");
  const auto tree_file = node->declare_parameter(
    "tree", ament_index_cpp::get_package_share_directory("jev_ai_bt") + "/bt/toy_patrol.xml");
  World world(static_cast<unsigned>(node->declare_parameter("seed", 0)));
  std::mutex world_mutex;

  BT::BehaviorTreeFactory factory;
  jev_ai_bt::register_nodes(factory);
  factory.registerSimpleAction(
    "Act", [&](BT::TreeNode & self) {
      const auto action = self.getInput<std::string>("action").value_or("");
      const auto note = self.getInput<std::string>("note").value_or("");
      std::lock_guard<std::mutex> lock(world_mutex);
      world.event.clear();
      const double now = node->now().seconds();
      if (action == "inspect" && world.task == "patrolling") {
        world.task = "inspecting";
        world.busy_until = now + 4;
      } else if (action == "halt_now" && world.task != "halted") {
        world.task = "halted";
        world.busy_until = now + 8;
      }
      RCLCPP_INFO(
        node->get_logger(), "act %s (%s) | %s, %.0f%%, detection: %s", action.c_str(),
        note.c_str(), world.task.c_str(), world.energy, world.detection.c_str());
      return BT::NodeStatus::SUCCESS;
    }, {BT::InputPort<std::string>("action"), BT::InputPort<std::string>("note")});

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  auto publish = [&]() {
      std::lock_guard<std::mutex> lock(world_mutex);
      blackboard->set("state", world.state());
      blackboard->set("event", world.event);
      blackboard->set("mask", world.detection == "none" ?
        jev_ai::ClassSets{{"next_action", {"inspect"}}} : jev_ai::ClassSets{});
      blackboard->set("require", world.energy <= 5 && world.task != "halted" ?
        jev_ai::Classes{{"next_action", "halt_now"}} : jev_ai::Classes{});
    };
  publish();
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(500), [&]() {
      {
        std::lock_guard<std::mutex> lock(world_mutex);
        world.step(node->now().seconds());
      }
      publish();
    });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&]() {executor.spin();});

  auto tree = factory.createTreeFromFile(tree_file, blackboard);
  while (rclcpp::ok()) {
    tree.tickOnce();
    tree.sleep(std::chrono::milliseconds(50));
  }
  std::signal(SIGINT, SIG_IGN);   // Ctrl-C arrives twice under launch (group + forward)
  tree.haltTree();
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
