#pragma once

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class WaitForOrder : public BT::SyncActionNode
{
public:
  WaitForOrder(const std::string &name, const BT::NodeConfiguration &config);

  ~WaitForOrder();

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  geometry_msgs::msg::PoseStamped last_pose_;
  std::atomic<bool> new_msg_;
  std::thread thread_;
};
