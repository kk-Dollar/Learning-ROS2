#include "master_ros2_interface/action/my_custom_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ActionClient : public rclcpp::Node
{
public:
    using MyCustomAction = master_ros2_interface::action::MyCustomAction;
    using GoalHandler = rclcpp_action::ClientGoalHandle<MyCustomAction>;

    ActionClient() : Node("action_client")
    {
        client_ = rclcpp_action::create_client<MyCustomAction>(this, "my_custom_action");
        while (!client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        }
        send_goal();
    }

private:
    rclcpp_action::Client<MyCustomAction>::SharedPtr client_;

    void send_goal()
    {
        if (!client_->action_server_is_ready())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        auto goal_msg = MyCustomAction::Goal();
        goal_msg.goal_value = 10;

        auto send_goal_options = rclcpp_action::Client<MyCustomAction>::SendGoalOptions();

        send_goal_options.feedback_callback = [this](GoalHandler::SharedPtr, const std::shared_ptr<const MyCustomAction::Feedback> feedback)
        {
            RCLCPP_INFO(get_logger(), "Feedback: %.2f", feedback->progress);
        };

        send_goal_options.result_callback = [this](const GoalHandler::WrappedResult &result)
        {
            RCLCPP_INFO(get_logger(), "Result: '%d', Status code: '%d'", result.result->result_value, static_cast<int>(result.code));
        };

        send_goal_options.goal_response_callback = [this](const GoalHandler::SharedPtr goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, starting execution");
            }
        };

        client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}