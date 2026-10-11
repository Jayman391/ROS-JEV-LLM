// formation_planner (B4): the coordinator's deterministic optimisers (DECISION_MODEL §7).
//   /swarm/allocate  (Allocate)   minimum-cost assignment (Hungarian): sectors, dispatch
//   /swarm/formation (Formation)  slots for a shape around a leader point, assigned to drones by
//                                 Hungarian on squared distance
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "swarm_control/hungarian.hpp"
#include "swarm_control/world.hpp"
#include "swarm_interfaces/srv/allocate.hpp"
#include "swarm_interfaces/srv/formation.hpp"

namespace swarm_control
{

using swarm_interfaces::srv::Allocate;
using swarm_interfaces::srv::Formation;

class FormationPlanner : public rclcpp::Node
{
public:
  FormationPlanner()
  : Node("formation_planner")
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
    formation_ = create_service<Formation>("/swarm/formation",
        [](const std::shared_ptr<Formation::Request> req, std::shared_ptr<Formation::Response> res) {
          const int n = static_cast<int>(req->positions.size());
          if (n == 0) {res->message = "no drones"; return;}
          const auto rel = formation_slots(req->shape, n, req->spacing);
          const double c = std::cos(req->heading), s = std::sin(req->heading);
          std::vector<geometry_msgs::msg::Point> slots(n);
          for (int i = 0; i < n; ++i) {
            slots[i].x = req->centre.x + c * rel[i].x - s * rel[i].y;
            slots[i].y = req->centre.y + s * rel[i].x + c * rel[i].y;
            slots[i].z = req->centre.z;
          }
          std::vector<double> cost(n * n);
          for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
              const auto & p = req->positions[i];
              cost[i * n + j] = std::pow(p.x - slots[j].x, 2) + std::pow(p.y - slots[j].y, 2);
            }
          }
          const auto a = hungarian(cost, n, n);
          for (int i = 0; i < n; ++i) {res->slots.push_back(slots[a[i]]);}
          res->success = true;
          res->message = req->shape + " formation, " + std::to_string(n) + " slots";
        });
  }

private:
  rclcpp::Service<Allocate>::SharedPtr allocate_;
  rclcpp::Service<Formation>::SharedPtr formation_;
};

}  // namespace swarm_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swarm_control::FormationPlanner>());
  rclcpp::shutdown();
  return 0;
}
