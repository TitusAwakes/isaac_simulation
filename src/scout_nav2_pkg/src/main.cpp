#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "rclcpp/rclcpp.hpp"

// Seus nós personalizados
#include "scout_nav2_pkg/wait_for_order.hpp"
#include "scout_nav2_pkg/send_goal_pose.hpp"
#include "scout_nav2_pkg/move_joints.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_runner_node");

  BT::BehaviorTreeFactory factory;

  // Registra o plugin padrão do Nav2 (ação de navegação)
  factory.registerFromPlugin("libnav2_navigate_to_pose_action_bt_node.so");

  // Registra seus nós personalizados
  factory.registerNodeType<WaitForOrder>("WaitForOrder");
  factory.registerNodeType<SendGoalPose>("SendGoalPose");
  factory.registerNodeType<MoveJoints>("MoveJoints");

  // Cria o Blackboard e define todas as chaves necessárias
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("server_timeout", std::chrono::milliseconds(10000));              // usado por NavigateToPose
  blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(5000));     // usado por NavigateToPose
  blackboard->set("bt_loop_duration", std::chrono::milliseconds(100));              // frequência do loop da árvore

  // Carrega a árvore XML
  auto tree = factory.createTreeFromFile("trees/tree_docked_charging.xml", blackboard);

  // Logger para debug
  BT::StdCoutLogger logger(tree);

  RCLCPP_INFO(node->get_logger(), "Starting Behavior Tree tick loop");

  rclcpp::Rate rate(10); // 10 Hz
  while (rclcpp::ok() && tree.tickRoot() == BT::NodeStatus::RUNNING)
  {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Behavior Tree finished");
  rclcpp::shutdown();
  return 0;
}

