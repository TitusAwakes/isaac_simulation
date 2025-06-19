#include "scout_nav2_pkg/wait_for_order.hpp"
#include <sstream>

WaitForOrder::WaitForOrder(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config), new_msg_(false)
{
  node_ = rclcpp::Node::make_shared("wait_for_order_node");

  sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/charging_goal", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        last_pose_ = *msg;
        new_msg_ = true;
      });

  thread_ = std::thread([this]()
                        { rclcpp::spin(node_); });
}

WaitForOrder::~WaitForOrder()
{
  rclcpp::shutdown();
  if (thread_.joinable())
    thread_.join();
}

BT::PortsList WaitForOrder::providedPorts()
{
  return {BT::OutputPort<std::string>("target_pose")};
}

BT::NodeStatus WaitForOrder::tick()
{
  if (new_msg_)
  {
    std::ostringstream ss;
    ss << last_pose_.pose.position.x << " "
       << last_pose_.pose.position.y << " "
       << last_pose_.pose.position.z;
    setOutput("target_pose", ss.str());
    new_msg_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}
