#include "scout_nav2_pkg/navigate_to_pose.hpp"

NavigateToPose::NavigateToPose(const std::string &name, const BT::NodeConfiguration &config)
: BT::StatefulActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("navigate_to_pose_bt_node");
  action_client_ = rclcpp_action::create_client<NavigateAction>(node_, "navigate_to_pose");
}

BT::PortsList NavigateToPose::providedPorts()
{
  return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")};
}

BT::NodeStatus NavigateToPose::onStart()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node_->get_logger(), "Action server não disponível");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal))
  {
    RCLCPP_ERROR(node_->get_logger(), "Falha ao obter a pose de meta");
    return BT::NodeStatus::FAILURE;
  }

  NavigateAction::Goal goal_msg;
  goal_msg.pose = goal;

  goal_handle_future_ = action_client_->async_send_goal(goal_msg);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPose::onRunning()
{
  if (goal_handle_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    rclcpp::spin_some(node_);
    return BT::NodeStatus::RUNNING;
  }

  goal_handle_ = goal_handle_future_.get();
  if (!goal_handle_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Falha ao enviar goal");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = action_client_->async_get_result(goal_handle_);

  if (result_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    rclcpp::spin_some(node_);
    return BT::NodeStatus::RUNNING;
  }

  auto result = result_future.get();

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_INFO(node_->get_logger(), "Navegação concluída com sucesso");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Navegação falhou ou foi cancelada");
    return BT::NodeStatus::FAILURE;
  }
}

void NavigateToPose::onHalted()
{
  RCLCPP_WARN(node_->get_logger(), "Nó NavigateToPose foi interrompido");
  // Opcional: cancelar a meta se quiser
  /*
  if (goal_handle_)
  {
    goal_handle_->async_cancel_goal();
  }
  */
}
