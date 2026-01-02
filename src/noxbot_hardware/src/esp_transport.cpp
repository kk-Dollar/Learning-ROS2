#include "noxbot_hardware/esp_transport.hpp"

EspTransport::EspTransport(rclcpp::Node * node) 
: node_(node), //these all are initiliser which is updated before the constructor call
  left_vel_(0.0),
  right_vel_(0.0),
  left_pos_(0.0),
  right_pos_(0.0),
  last_feedback_time_(node->now()),
  timeout_(rclcpp::Duration::from_seconds(0.2)),
  initialized_(false)
  {
     //publisher: ROS-->ESP32
     cmd_pub_=node_->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_commands",10);

     //subscriber: ESP32-->ROS

     fb_sub_=node_->create_subscription<std_msgs::msg::Float64MultiArray>("/wheel_feedback",10,std::bind(&EspTransport::feedbackCallback,this,std::placeholders::_1));

  }

  bool EspTransport::init()
  {
     initialized_=false;
     RCLCPP_INFO(node_->get_logger(),"Waiting for ESP32 feedback...");

     rclcpp::Time start= node_->now();

     while((node_->now()-start)<rclcpp::Duration::from_seconds(1.0))
     {
        rclcpp::spin_some(node_->get_node_base_interface());

        if((node_->now()-last_feedback_time_)<timeout_)
        {
            initialized_=true;
            RCLCPP_INFO(node_->get_logger(), "ESP32 Connection established");
            return true;
        }
     }
     RCLCPP_WARN(node_->get_logger(),"ESP32 not responding - starting in SAFE MODE");
     return false;
  }
  bool EspTransport::isAlive() const 
  {
    return (node_->now()-last_feedback_time_)<timeout_;
  }
  void EspTransport::sendWheelCommands(double left_rad_s, double right_rad_s)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(2);
    msg.data[0]=left_rad_s;
    msg.data[1]=right_rad_s;
    
    cmd_pub_->publish(msg);
  }
  void EspTransport::feedbackCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if(msg->data.size()<4)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,"wheel-feedback message malformed");
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    left_vel_=msg->data[0]; // rad/s
    right_vel_=msg->data[1];
    left_pos_=msg->data[2];  //rad
    right_pos_=msg->data[3];

    last_feedback_time_=node_->now();
  }
  double EspTransport::getLeftVelocity() const  //const func matlb: ye functino object ka data change nahi karega
  {
     std::lock_guard<std::mutex> lock(data_mutex_);
     return left_vel_;
  }
  double EspTransport::getRightVelocity() const  //const func matlb: ye functino object ka data change nahi karega
  {
     std::lock_guard<std::mutex> lock(data_mutex_);
     return right_vel_;
  }
  double EspTransport::getLeftPosition() const  //const func matlb: ye functino object ka data change nahi karega
  {
     std::lock_guard<std::mutex> lock(data_mutex_);
     return left_pos_;
  }
  double EspTransport::getRightPosition() const  //const func matlb: ye functino object ka data change nahi karega
  {
     std::lock_guard<std::mutex> lock(data_mutex_);
     return right_pos_;
  }