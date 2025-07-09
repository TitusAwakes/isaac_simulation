#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "scout_nav2_pkg/wait_for_order.hpp"
#include "scout_nav2_pkg/send_goal_pose.hpp"
#include "scout_nav2_pkg/move_joints.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_runner_node");

  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<WaitForOrder>("WaitForOrder");
  factory.registerNodeType<SendGoalPose>("SendGoalPose");
  factory.registerNodeType<MoveJoints>("MoveJoints");

  // Compartilha o node via Blackboard
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  auto tree = factory.createTreeFromFile("trees/tree_docked_charging.xml", blackboard);

  rclcpp::Rate rate(10);
  while (rclcpp::ok() && tree.tickRoot() == BT::NodeStatus::RUNNING) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

