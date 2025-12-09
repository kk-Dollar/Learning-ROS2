#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "master_ros2_interface/msg/custom_msg.hpp"

using namespace std;

class PublisherNode : public rclcpp::Node{
public:
    PublisherNode(): Node("publisher_node_param"){
        auto qos_profile= rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
        publisher_ = this->create_publisher<std_msgs::msg::String>("std_string_topic",qos_profile);
        custom_publisher_=this->create_publisher<master_ros2_interface ::msg::CustomMsg>("custom_topic",qos_profile);
        timer_=this->create_wall_timer(std::chrono::seconds(1),bind(&PublisherNode::publish_messages,this));
        this->declare_parameter<string>("custom_string","Hello world!");
        this->declare_parameter<int>("custom_number",42);

    }
private:    

    void publish_messages(){
        auto string_msg= std_msgs::msg::String();
        string_msg.data="Hello, world!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'",string_msg.data.c_str());
        publisher_->publish(string_msg);

        //
        string custom_string_param;
        int custom_number_param;

        this->get_parameter("custom_string",custom_string_param);
        this->get_parameter("custom_number",custom_number_param);
        auto custom_msg=master_ros2_interface::msg::CustomMsg();
        custom_msg.data=custom_string_param;
        custom_msg.number=custom_number_param;
        RCLCPP_INFO(this->get_logger(),"Publishing custom message: data= '%s', number='%d'",custom_msg.data.c_str(),custom_msg.number);
        custom_publisher_->publish(custom_msg);
    } 
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<master_ros2_interface::msg::CustomMsg>::SharedPtr custom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc , char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());

    rclcpp::shutdown();
}

