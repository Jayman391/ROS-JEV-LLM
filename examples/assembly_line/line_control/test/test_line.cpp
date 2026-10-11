// The line world without ROS: pallets flow, fasten needs both arms, defer laps, rework waits for
// a route, jams fail the op and clear.
#include <gtest/gtest.h>

#include "line_control/line.hpp"

namespace {

line::Params params() {
  line::Params p;
  p.arms = {"arm_a", "arm_b", "arm_c", "arm_d", "arm_e"};
  p.arm_station = {1, 2, 3, 3, 4};
  p.arm_tools = {"pick,screw,insert", "weld,screw", "pick,insert,hold", "hold,screw,glue",
                 "glue,weld,pick"};
  p.products = {"A", "B", "C"};
  p.recipes = {"pick,screw,weld,glue", "pick,insert,fasten,screw,weld",
               "pick,weld,glue,insert,fasten,screw"};
  p.op_kinds = {"pick", "screw", "weld", "glue", "insert", "fasten"};
  p.op_time_s = {6, 8, 12, 10, 7, 10};
  p.qa_sigma = {0.08, 0.05, 0.06, 0.08, 0.08, 0.05};
  p.jam_p0 = p.jam_p1 = 0.0;
  p.dropouts = 0;
  p.p_grip_slip = p.p_grip_marginal = p.p_misalign = 0.0;
  return p;
}

// every idle arm starts the op pending for it, unless it is in `skip`
void run(line::Line& l, double seconds, const std::set<std::string>& skip = {}) {
  for (double t = 0; t < seconds; t += 0.25) {
    l.step(0.25);
    for (auto& a : l.arms) {
      if (a.state != "idle" || skip.count(a.name)) continue;
      line::Part* pt = l.pending_for(a);
      if (pt) l.operate(a.name, "op", pt->id, static_cast<int>(pt->k));
    }
  }
}

int at_qa(line::Line& l) { return l.qa_ready.empty() ? -1 : l.qa_ready.front(); }

}  // namespace

TEST(Line, PartFlowsToQaAndShips) {
  line::Line l(params());
  int id = -1;
  ASSERT_EQ(l.release("A", {"arm_a", "arm_b", "arm_b", "arm_e"}, &id), "");
  EXPECT_EQ(l.release("B", {"arm_a", "arm_a", "arm_d+arm_c", "arm_d", "arm_e"}, nullptr),
            "feeder busy");
  run(l, 120);
  ASSERT_EQ(at_qa(l), id);
  auto r = l.qa_readings(id);
  ASSERT_EQ(r.size(), 4u);
  EXPECT_EQ(r[2].kind, "weld");
  EXPECT_EQ(r[2].arm, "arm_b");
  EXPECT_EQ(l.dispose(id, "pass", 0), "done");
  ASSERT_EQ(l.left.size(), 1u);
  EXPECT_EQ(l.left[0].outcome, "shipped");
  EXPECT_FALSE(l.left[0].defective);           // no wear, no bad grips
  EXPECT_EQ(l.shipped["A"], 1);
}

TEST(Line, RouteValidation) {
  line::Line l(params());
  EXPECT_NE(l.release("A", {"arm_a", "arm_c", "arm_b", "arm_e"}, nullptr), "");  // c can't screw
  EXPECT_NE(l.release("A", {"arm_a", "arm_b", "arm_b"}, nullptr), "");           // too short
  // fasten: primary screws, partner holds, same station
  EXPECT_NE(l.release("B", {"arm_a", "arm_a", "arm_c+arm_d", "arm_d", "arm_e"}, nullptr), "");
  EXPECT_NE(l.release("B", {"arm_a", "arm_a", "arm_d", "arm_d", "arm_e"}, nullptr), "");
  EXPECT_EQ(l.release("B", {"arm_a", "arm_c", "arm_d+arm_c", "arm_d", "arm_e"}, nullptr), "");
}

TEST(Line, FastenNeedsBothArms) {
  line::Line l(params());
  int id = -1;
  ASSERT_EQ(l.release("B", {"arm_a", "arm_a", "arm_d+arm_c", "arm_d", "arm_e"}, &id), "");
  run(l, 120, {"arm_c"});                      // the holder never comes
  line::Arm& d = *l.arm("arm_d");
  EXPECT_EQ(d.state, "coop_wait");
  EXPECT_GT(d.coop_wait, 30.0);
  EXPECT_EQ(l.parts.at(id).k, 2u);
  run(l, 120);
  EXPECT_EQ(at_qa(l), id);
  EXPECT_EQ(l.parts.at(id).ops[2].did, "arm_d+arm_c");
}

TEST(Line, DeferLapsAndComesBack) {
  line::Line l(params());
  int id = -1;
  ASSERT_EQ(l.release("A", {"arm_a", "arm_b", "arm_b", "arm_e"}, &id), "");
  run(l, 30, {"arm_b"});                       // pick + travel; screw is pending at S2
  ASSERT_EQ(l.parts.at(id).k, 1u);
  ASSERT_EQ(l.where(l.parts.at(id)), "S2");
  EXPECT_EQ(l.operate("arm_b", "defer", id, 1), "done");
  run(l, 2, {"arm_b"});
  EXPECT_EQ(l.parts.at(id).st, line::Part::LANE);
  run(l, 200);
  EXPECT_EQ(at_qa(l), id);
}

TEST(Line, ReworkWaitsForARoute) {
  line::Line l(params());
  int id = -1;
  ASSERT_EQ(l.release("A", {"arm_a", "arm_b", "arm_b", "arm_e"}, &id), "");
  run(l, 120);
  ASSERT_EQ(at_qa(l), id);
  l.qa_ready.clear();
  EXPECT_EQ(l.dispose(id, "rework", 2), "done");
  run(l, 60);
  EXPECT_EQ(l.where(l.parts.at(id)), "feeder");   // unrouted redo: waits
  EXPECT_FALSE(l.feeder_free());
  ASSERT_EQ(l.route(id, 4, {"arm_e"}), "");
  run(l, 120);
  ASSERT_EQ(at_qa(l), id);
  EXPECT_EQ(l.dispose(id, "pass", 0), "done");
  EXPECT_EQ(l.left[0].part.reworks, 1);
  EXPECT_EQ(l.left[0].part.ops[2].did, "arm_e");  // the redo replaced the weld
}

TEST(Line, JamFailsTheOpAndClears) {
  line::Params p = params();
  p.jam_p0 = 1.0;
  p.jam_reset_min = p.jam_reset_max = 40.0;
  line::Line l(p);
  int id = -1;
  ASSERT_EQ(l.release("A", {"arm_a", "arm_b", "arm_b", "arm_e"}, &id), "");
  run(l, 20);
  line::Arm& a = *l.arm("arm_a");
  EXPECT_EQ(a.state, "jammed");
  EXPECT_EQ(a.jams, 1);
  bool saw_jam = false;
  for (const auto& e : l.ended) saw_jam |= e.arm == "arm_a" && e.message == "jam";
  EXPECT_TRUE(saw_jam);
  EXPECT_EQ(l.parts.at(id).k, 0u);
  EXPECT_EQ(l.operate("arm_a", "op", id, 0), "rejected: jammed");
  EXPECT_GT(l.view(a).jam_clear, 0.0);
  run(l, 45, {"arm_a"});
  EXPECT_EQ(a.state, "idle");
}

TEST(Line, ToolChangeResetsWear) {
  line::Line l(params());
  line::Arm& a = *l.arm("arm_a");
  a.wear = 0.8;
  EXPECT_EQ(l.operate("arm_a", "tool_change", -1, -1), "");
  run(l, 30, {"arm_a"});
  EXPECT_EQ(a.wear, 0.0);
  EXPECT_EQ(a.state, "idle");
}
