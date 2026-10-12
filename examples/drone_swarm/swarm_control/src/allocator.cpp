// allocator (B4): the coordinator's deterministic optimiser (DECISION_MODEL §7).
//   /swarm/allocate  (Allocate)   minimum-cost assignment (Hungarian): sectors, dispatch
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "swarm_control/hungarian.hpp"
#include "swarm_interfaces/srv/allocate.hpp"

namespace swarm_control
{

using swarm_interfaces::srv::Allocate;

class Allocator : public rclcpp::Node
{
public:
  Allocator()
  : Node("allocator")
  {
    allocate_ = create_service<Allocate>("/swarm/allocate",
        [](const std::shared_ptr<Allocate::Request> req, std::shared_ptr<Allocate::Response> res) {
          const int r = static_cast<int>(req->rows), c = static_cast<int>(req->cols);
          if (r * c != static_cast<int>(req->cost.size()) || r == 0 || c == 0) {return;}
          res->assignment = hungarian(req->cost, r, c);
          for (int i = 0; i < r; ++i) {
            if (res->assignment[i] >= 0) {res->total += req->cost[i * c + res->assignment[i]];}
          }
        });
  }

private:
  rclcpp::Service<Allocate>::SharedPtr allocate_;
};

}  // namespace swarm_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swarm_control::Allocator>());
  rclcpp::shutdown();
  return 0;
}
