#include "master_ros2_interface/srv/concat_string.hpp"
#include "rclcpp/rclcpp.hpp"

class StringConcatService : public rclcpp::Node{
public:
     StringConcatService(): Node("concat_sting_server"){
        RCLCPP_INFO(this->get_logger(),"Concat String server started....");
        service_=this->create_service<master_ros2_interface::srv::ConcatString>("concat_string",std::bind(&StringConcatService::server_callback,this,std::placeholders::_1,std::placeholders::_2));
     }

private:
     void server_callback(const std::shared_ptr<master_ros2_interface::srv::ConcatString::Request> request, const std::shared_ptr<master_ros2_interface::srv::ConcatString::Response> response)  {
         response->concatenated_str= request->str1+ request->str2;
         RCLCPP_INFO(this->get_logger(),"String_1: '%s' , String_2: '%s' , concated_string: '%s'",request->str1.c_str(),request->str2.c_str(),response->concatenated_str.c_str());
     }   
     rclcpp::Service<master_ros2_interface::srv::ConcatString>::SharedPtr service_;
} 
;

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<StringConcatService>());
    rclcpp::shutdown();
    return 0;
}
