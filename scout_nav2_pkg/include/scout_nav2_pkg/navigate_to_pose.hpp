#ifndef SCOUT_NAV2_PKG_NAVIGATE_TO_POSE_HPP_
#define SCOUT_NAV2_PKG_NAVIGATE_TO_POSE_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using NavigateAction = nav2_msgs::action::NavigateToPose;

class NavigateToPose : public BT::StatefulActionNode
{
public:
  NavigateToPose(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateAction>::SharedPtr action_client_;

  std::shared_future<rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr> goal_handle_future_;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateAction>> goal_handle_;
};

#endif  // SCOUT_NAV2_PKG_NAVIGATE_TO_POSE_HPP_
