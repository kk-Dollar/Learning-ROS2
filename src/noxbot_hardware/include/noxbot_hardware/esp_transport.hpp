#pragma once // talk to compiler to include once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <mutex> //allow lock and unlock mechanism to allow only one tread to see or update the data

class EspTransport
{
 public:
   explicit EspTransport(rclcpp::Node * node); //explict use karne se auto constructor call nahi hoga 
   
   //handshake
   bool init();
   bool isAlive() const;

   //command(ROS)--> ESP32
   void sendWheelCommands(double left_rad_s, double right_rad_s);

   //feedback(ESP32)-->ROS
   double getLeftVelocity() const;
   double getRightVelocity() const;
   double getLeftPosition() const;
   double getRightPosition() const;


 private:
    //ROS callback
    void feedbackCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    //ROS handle
    rclcpp::Node * node_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr fb_sub_;

    //state storage
    double left_vel_;
    double right_vel_;
    double left_pos_;
    double right_pos_;

    //timing / safety

    rclcpp::Time last_feedback_time_;
    rclcpp::Duration timeout_;
    bool initialized_;

    mutable std::mutex data_mutex_;

};