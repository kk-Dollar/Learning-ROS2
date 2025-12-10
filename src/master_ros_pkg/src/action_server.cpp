#include "master_ros2_interface/action/my_custom_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ActionServer: public rclcpp::Node{
    public:
        using MyCustomAction= master_ros2_interface::action::MyCustomAction;
        using GoalHandle= rclcpp_action::ServerGoalHandle<MyCustomAction>;
       ActionServer(): Node("action_server"){
         action_server_= rclcpp_action::create_server<MyCustomAction>(
          this,
          "my_custom_action",
          std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
          std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
          std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));
       }
    private:   

    std::shared_ptr<rclcpp_action::Server<MyCustomAction>> action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, 
       std::shared_ptr<const MyCustomAction::Goal> goal ){
         RCLCPP_INFO(get_logger(),"The goal received with data: '%d'", goal->goal_value);
         return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } 
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle){
         RCLCPP_WARN(get_logger(),"Receiver request to cancel the Goal");
         return rclcpp_action::CancelResponse::ACCEPT;
    }  
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle){
        std::thread{std::bind(&ActionServer::execute,this,std::placeholders::_1),goal_handle}.detach();
    }
    void execute (const std::shared_ptr<GoalHandle> goal_handle){
       const auto goal= goal_handle->get_goal();
       auto result= std::make_shared<MyCustomAction::Result>();
       auto feedback= std::make_shared<MyCustomAction::Feedback>();

       for(int i=0;i<goal->goal_value;i++)
       {
        feedback->progress=i;
        goal_handle->publish_feedback(feedback);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
       }
       result->result_value=goal->goal_value;
       goal_handle->succeed(result);
       RCLCPP_INFO(get_logger(),"Goal Succeeded");
    }
    
};

int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    auto node= std::make_shared<ActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
