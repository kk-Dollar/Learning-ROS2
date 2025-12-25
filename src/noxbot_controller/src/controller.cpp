#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode() : Node("controller_node")
  {
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&ControllerNode::cmd_callback, this, std::placeholders::_1));

    left_pub_  = create_publisher<std_msgs::msg::Int32>("/left_wheel_pwm", 10);
    right_pub_ = create_publisher<std_msgs::msg::Int32>("/right_wheel_pwm", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),   // 50 Hz
      std::bind(&ControllerNode::control_loop, this));

    RCLCPP_INFO(get_logger(), "Controller running at 50 Hz");
  }

private:
  /* ----------- Callbacks ----------- */

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_v_ = msg->linear.x;
    last_w_ = msg->angular.z;
    cmd_received_ = true;
  }

  void control_loop()
  {
    if (!cmd_received_) return;

    // Differential drive kinematics
    double v_right = last_v_ + (last_w_ * wheel_sep_ / 2.0);
    double v_left  = last_v_ - (last_w_ * wheel_sep_ / 2.0);

    int pwm_right = vel_to_pwm(v_right);
    int pwm_left  = vel_to_pwm(v_left);

    std_msgs::msg::Int32 lmsg, rmsg;
    lmsg.data = pwm_left;
    rmsg.data = pwm_right;

    left_pub_->publish(lmsg);
    right_pub_->publish(rmsg);
  }

  /* ----------- Helpers ----------- */

  int vel_to_pwm(double v)
  {
    const int MAX_PWM = 255;
    const int MIN_PWM = 80;
    const double MAX_VEL = 1.0;

    if (std::abs(v) < 0.01) return 0;

    int pwm = static_cast<int>((v / MAX_VEL) * MAX_PWM);
    pwm = std::clamp(pwm, -MAX_PWM, MAX_PWM);

    // deadzone compensation
    if (pwm > 0 && pwm < MIN_PWM) pwm = MIN_PWM;
    if (pwm < 0 && pwm > -MIN_PWM) pwm = -MIN_PWM;

    return pwm;
  }

  /* ----------- ROS ----------- */

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* ----------- State ----------- */

  double last_v_ = 0.0;
  double last_w_ = 0.0;
  bool cmd_received_ = false;

  const double wheel_sep_ = 0.15;  // meters
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
