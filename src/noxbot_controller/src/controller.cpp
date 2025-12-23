#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>

class ControllerNode : public rclcpp::Node{
public:    
      ControllerNode():Node("controller"){
        
        subscriber_node_= this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,std::bind(&ControllerNode::Twist_callback,this,std::placeholders::_1));

        right_wheel_publisher_=this->create_publisher<std_msgs::msg::Int32>("/right_wheel_pwm",10);
        left_wheel_publisher_=this->create_publisher<std_msgs::msg::Int32>("/left_wheel_pwm",10);
        last_cmd_time_=this->now();
        watchdog_timer_= this->create_wall_timer(std::chrono::milliseconds(50),std::bind(&ControllerNode::watchdog_callback,this));
      }
private:

  double clamp_vel(double x, double min, double max, double e){
    if(std::abs(x)<=e) return 0;
    if(x<min) return min;
    else if(x>max) return max;
    else return x;
  }

  void Twist_callback(const geometry_msgs::msg::Twist::SharedPtr twist){
    last_cmd_time_=this->now();
    cmd_received_=true;
    double v= twist->linear.x;
    double w= twist->angular.z;

    double L= 0.15;
    const double max_wheel_vel=1.0;
    const double epsilon=0.01;
    const int max_pwm=255;

    auto v_right= v + (w* (L/2));
    auto v_left=v- (w* (L/2));


    auto right_wheel_vel= clamp_vel(v_right,-max_wheel_vel,+max_wheel_vel,epsilon);
    auto left_wheel_vel= clamp_vel(v_left,-max_wheel_vel,+max_wheel_vel,epsilon);

  
    auto right_wheel_pwm= std_msgs::msg::Int32();
    auto left_wheel_pwm= std_msgs::msg::Int32();
    
    
    right_wheel_pwm.data= std::clamp((int)(std::round((right_wheel_vel/max_wheel_vel)*max_pwm)),-max_pwm,+max_pwm);
    left_wheel_pwm.data= std::clamp((int)(std::round((left_wheel_vel/max_wheel_vel)*max_pwm)),-max_pwm,+max_pwm);
    
    right_wheel_publisher_->publish(right_wheel_pwm);
    left_wheel_publisher_->publish(left_wheel_pwm);
  }
  void watchdog_callback()
  {
    const double timeout_sec=0.5;
     if(!cmd_received_)
     return ;
     double elapsed= (this->now()-last_cmd_time_).seconds();
     if(elapsed>timeout_sec)
     {
      std_msgs::msg::Int32 stop_msg;
      stop_msg.data=0;
      right_wheel_publisher_->publish(stop_msg);
      left_wheel_publisher_->publish(stop_msg);
      cmd_received_=false;
      RCLCPP_WARN_ONCE(get_logger(), "Watchdog stop triggered");

     }
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_node_;    
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_wheel_publisher_; 
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_wheel_publisher_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_cmd_time_;
  bool cmd_received_=false;
  
};

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}