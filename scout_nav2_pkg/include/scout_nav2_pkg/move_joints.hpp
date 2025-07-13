#ifndef SCOUT_NAV2_PKG_MOVE_JOINTS_HPP_
#define SCOUT_NAV2_PKG_MOVE_JOINTS_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <vector>

class MoveJoints : public BT::SyncActionNode
{
public:
  MoveJoints(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
};

#endif  // SCOUT_NAV2_PKG_MOVE_JOINTS_HPP_

