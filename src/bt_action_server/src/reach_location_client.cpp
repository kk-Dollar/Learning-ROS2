#include "bt_action_server/action/reach_location.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ReachLocationClientAction : public rclcpp::Node
{
public:
  using Action = bt_action_server::action::ReachLocation;
  using GoalHandler = rclcpp_action::ClientGoalHandle<Action>;

  ReachLocationClientAction() : Node("reach_location_action_client")
  {
    client_ = rclcpp_action::create_client<Action>(this, "reach_location");
    while (!client_->wait_for_action_server(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the action server. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }
    send_goal();
  }
  ~ReachLocationClientAction()
  {
    if(goal_handle_&&rclcpp::ok())
    {
        client_->async_cancel_goal(goal_handle_);
    }
  }

private:
  rclcpp_action::Client<Action>::SharedPtr client_;
  GoalHandler::SharedPtr goal_handle_;

  void send_goal()
  {
    if (!client_->action_server_is_ready())
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    auto goal_msg = Action::Goal();
    goal_msg.x = 4;
    goal_msg.y = 4;
    goal_msg.timeout = 10;

    auto send_goal_options = rclcpp_action::Client<Action>::SendGoalOptions();

    send_goal_options.feedback_callback = [this](GoalHandler::SharedPtr,
                                                 const std::shared_ptr<const Action::Feedback> feedback) {
      RCLCPP_INFO(get_logger(), "Feedback: current_x: %.2f , current_y: %.2f", feedback->current_x,
                  feedback->current_y);
    };

    send_goal_options.result_callback = [this](const GoalHandler::WrappedResult& result) {
      switch (result.code)
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          break;
      }
      rclcpp::shutdown();
    };

    send_goal_options.goal_response_callback = [this](const GoalHandler::SharedPtr goal_handle) {
      if (!goal_handle)
      {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      }
      else
      {
        goal_handle_ = goal_handle;
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, starting execution");
      }
    };

    client_->async_send_goal(goal_msg, send_goal_options);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReachLocationClientAction>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}