#include "scout_nav2_pkg/move_joints.hpp"

MoveJoints::MoveJoints(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("move_joints_bt_node");
  pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);
}

BT::PortsList MoveJoints::providedPorts()
{
  return {
    BT::InputPort<std::vector<double>>("joint_values")
  };
}

BT::NodeStatus MoveJoints::tick()
{
  std::vector<double> joint_values;
  if (!getInput("joint_values", joint_values)) {
    RCLCPP_ERROR(node_->get_logger(), "No joint values provided to MoveJoints");
    return BT::NodeStatus::FAILURE;
  }

  if (joint_values.size() != 8) {
    RCLCPP_ERROR(node_->get_logger(), "Expected 8 joint values, got %zu", joint_values.size());
    return BT::NodeStatus::FAILURE;
  }

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8"
  };

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = joint_values;
  point.time_from_start = rclcpp::Duration::from_seconds(3.0);

  traj_msg.points.push_back(point);

  pub_->publish(traj_msg);

  RCLCPP_INFO(node_->get_logger(), "Published joint trajectory command");

  return BT::NodeStatus::SUCCESS;
}

