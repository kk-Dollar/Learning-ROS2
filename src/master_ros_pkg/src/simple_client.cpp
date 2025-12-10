#include "master_ros2_interface/srv/concat_string.hpp"
#include "rclcpp/rclcpp.hpp"

class ConcatStringClient : rclcpp::Node{
    public:
         ConcatStringClient(): Node("concat_string_client"){
            client_=this->create_client<master_ros2_interface::srv::ConcatString>("concat_string");
         }

    
      void send_request(const std::string &str1, const std::string &str2)
      {
        while(!client_->wait_for_service(std::chrono::seconds(1))){
            if (!rclcpp::ok()) {
         RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
         return;
         }
            RCLCPP_WARN(this->get_logger(), "Server is not available, retrying....");
        }
        auto request= std::make_shared<master_ros2_interface::srv::ConcatString::Request>();
        request->str1=str1;
        request->str2=str2;
        auto future_result= client_->async_send_request(request);
        if(rclcpp::spin_until_future_complete(this->get_node_base_interface(),future_result)==rclcpp::FutureReturnCode::SUCCESS){
            RCLCPP_INFO(this->get_logger(),"Result: concatenated_str='%s'",future_result.get()->concatenated_str.c_str());
        }
      } 
      private:
      rclcpp::Client<master_ros2_interface::srv::ConcatString>::SharedPtr client_;     
}
;

int main(int argc ,char** argv){
    rclcpp::init(argc,argv);
    auto client= std::make_shared<ConcatStringClient>();
    client->send_request("hello ","krishna");

    rclcpp::shutdown();
    return 0;
}