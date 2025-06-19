// Placeholder since I don't know what to put here yet

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "scout_nav2_pkg/wait_for_order.hpp"
#include "scout_nav2_pkg/navigate_to_pose.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<WaitForOrder>("WaitForOrder");
  factory.registerNodeType<NavigateToPose>("NavigateToPose");

  auto tree = factory.createTreeFromFile("trees/tree_wireless_charging.xml");

  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    rclcpp::spin_some(rclcpp::Node::make_shared("bt_tick"));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;
}
