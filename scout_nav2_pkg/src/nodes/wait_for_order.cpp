#include "scout_nav2_pkg/wait_for_order.hpp"

WaitForOrder::WaitForOrder(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config), new_msg_(false)
{
  config.blackboard->get("node", node_);

  sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/charging_goal", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      last_pose_ = *msg;
      new_msg_ = true;
    });
}

BT::PortsList WaitForOrder::providedPorts()
{
  return { BT::OutputPort<geometry_msgs::msg::PoseStamped>("target_pose") };
}

BT::NodeStatus WaitForOrder::tick()
{
  RCLCPP_INFO(node_->get_logger(), "Aguardando nova meta...");

  while (rclcpp::ok() && !new_msg_) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (!rclcpp::ok()) {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("target_pose", last_pose_);
  new_msg_ = false;

  RCLCPP_INFO(node_->get_logger(), "Nova meta recebida: [%.2f, %.2f]",
              last_pose_.pose.position.x, last_pose_.pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

