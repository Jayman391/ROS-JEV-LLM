// jev_ai BehaviorTree.CPP v4 nodes (FRAMEWORK §2, A8): Decide, JevAsk, Reason.
//
//   <Decide question_set="next_step" state="{state}" mask="{mask}" question="next_action"
//           selected="{next_action}" phase="{phase}"/>
//   <JevAsk question="is_person" type="noul" instructions="Is it a person?" state="{state}"
//           answer="{is_person}"/>
//   <Reason system="You are ..." context="{ctx}" text="{reply}"/>
//
// The ROS node comes from the blackboard entry "node" (rclcpp::Node::SharedPtr), the
// BehaviorTree.ROS2 convention; spin it on another thread. String ports parse as
//   state / require:  "key=value;key=value"
//   mask:             "question=class,class;question=class"
// so trees can be written with literals, and C++ apps can put the typed values on the blackboard.
#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "jev_ai/clients.hpp"

namespace BT
{
template<>
jev_ai::State convertFromString<jev_ai::State>(StringView str);   // also jev_ai::Classes
template<>
jev_ai::ClassSets convertFromString<jev_ai::ClassSets>(StringView str);
}  // namespace BT

namespace jev_ai_bt
{
/// Registers Decide, JevAsk and Reason (also done by the plugin entry point).
void register_nodes(BT::BehaviorTreeFactory & factory);
}  // namespace jev_ai_bt
