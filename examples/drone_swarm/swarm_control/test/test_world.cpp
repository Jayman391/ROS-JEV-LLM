// gtest (B3/B4): the pure world model: Hungarian, ray casting, energy, safety filter.
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "swarm_control/hungarian.hpp"
#include "swarm_control/world.hpp"

using swarm_control::V3;

TEST(Hungarian, SquareOptimum)
{
  // the classic 3x3: optimum 0->1, 1->0, 2->2 with cost 5
  const std::vector<double> c = {4, 1, 3, 2, 0, 5, 3, 2, 2};
  const auto a = swarm_control::hungarian(c, 3, 3);
  EXPECT_EQ(a, (std::vector<int>{1, 0, 2}));
}

TEST(Hungarian, Rectangular)
{
  // 3 drones, 2 sectors: the cheapest two drones get them, one stays unassigned
  const std::vector<double> c = {10, 10, 1, 9, 9, 1};
  const auto a = swarm_control::hungarian(c, 3, 2);
  EXPECT_EQ(a, (std::vector<int>{-1, 0, 1}));
  const auto b = swarm_control::hungarian({5, 1, 2, 7}, 1, 4);    // 1 drone, 4 tracks
  EXPECT_EQ(b, (std::vector<int>{1}));
}

TEST(World, RayCylinder)
{
  const swarm_control::Cylinder c{10, 0, 2, 30};
  EXPECT_NEAR(swarm_control::ray_cylinder(0, 0, 10, 0.0, c), 8.0, 1e-9);
  EXPECT_TRUE(std::isinf(swarm_control::ray_cylinder(0, 0, 10, M_PI, c)));   // behind
  EXPECT_TRUE(std::isinf(swarm_control::ray_cylinder(0, 0, 35, 0.0, c)));    // above it
}

TEST(World, EnergyMargin)
{
  swarm_control::Params p;
  const V3 home{0, 0, 0}, calm{0, 0, 0}, head{6, 0, 0};    // blowing +x: against the way home
  const double near = swarm_control::rtb_wh(p, {50, 0, 30}, home, calm);
  const double far = swarm_control::rtb_wh(p, {300, 0, 30}, home, calm);
  const double windy = swarm_control::rtb_wh(p, {300, 0, 30}, home, head);
  EXPECT_LT(near, far);
  EXPECT_LT(far, windy);           // flying home into a headwind costs more
  EXPECT_NEAR(swarm_control::energy_margin(p, p.E_0, 0.0), 1.0 - p.reserve, 1e-9);
  EXPECT_NEAR(swarm_control::link_quality(p, p.R_c), 0.5, 1e-9);
}

TEST(Safety, NfzDeflectsAndSaturates)
{
  swarm_control::Params p;
  p.nfz.push_back({100, 0, 20, INFINITY});
  // heading straight at the NFZ from 30 m out (inside the influence zone)
  auto s = swarm_control::safety_filter(p, {70, 0, 30}, {5, 0, 0}, {}, 0, 0, {}, 5.0, 2.0);
  EXPECT_LE(s.v.x, 1e-9);                       // no inward motion left
  EXPECT_GT(std::abs(s.v.y), 1.0);              // redirected tangentially
  EXPECT_LE(s.v.hnorm(), 5.0 + 1e-9);
  // separation: another drone 2 m ahead pushes back
  auto t = swarm_control::safety_filter(p, {0, 0, 30}, {0, 0, 0}, {}, 0, 0, {{2, 0, 30}}, 5, 2);
  EXPECT_LT(t.v.x, 0.0);
  EXPECT_EQ(t.reflexes, (std::vector<std::string>{"separation"}));
  // geofence: at the AO's far edge moving out
  auto g = swarm_control::safety_filter(p, {p.ao[1] + 9, 0, 30}, {3, 0, 0}, {}, 0, 0, {}, 5, 2);
  EXPECT_LT(g.v.x, 0.0);
}
