#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

class RosSerialNode : public rclcpp::Node
{
public:
  RosSerialNode() : Node("ros_serial_node")
  {
    left_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/left_wheel_pwm", 10,
      std::bind(&RosSerialNode::left_cb, this, std::placeholders::_1));

    right_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/right_wheel_pwm", 10,
      std::bind(&RosSerialNode::right_cb, this, std::placeholders::_1));

    open_serial("/dev/ttyUSB0");

    timer_ = create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&RosSerialNode::send_serial, this));

    RCLCPP_INFO(get_logger(), "ros_serial started (continuous send)");
  }

  ~RosSerialNode()
  {
    if (serial_fd_ >= 0)
      close(serial_fd_);
  }

private:
  /* ---------------- ROS callbacks ---------------- */

  void left_cb(const std_msgs::msg::Int32::SharedPtr msg)
  {
    left_pwm_ = msg->data;
    left_ready_ = true;
  }

  void right_cb(const std_msgs::msg::Int32::SharedPtr msg)
  {
    right_pwm_ = msg->data;
    right_ready_ = true;
  }

  /* ---------------- Serial ---------------- */

  void open_serial(const char *port)
  {
    serial_fd_ = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial port %s", port);
      rclcpp::shutdown();
      return;
    }

    struct termios tty{};
    tcgetattr(serial_fd_, &tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tcsetattr(serial_fd_, TCSANOW, &tty);

    RCLCPP_INFO(get_logger(), "Serial opened on %s @115200", port);
  }

  /* ---------------- Helpers ---------------- */

  int apply_deadzone(int pwm)
  {
    const int MIN_PWM = 80;   // experimentally safe
    if (pwm == 0) return 0;
    if (pwm > 0 && pwm < MIN_PWM) return MIN_PWM;
    if (pwm < 0 && pwm > -MIN_PWM) return -MIN_PWM;
    return pwm;
  }

  void send_serial()
  {
    if (serial_fd_ < 0) return;
    if (!left_ready_ || !right_ready_) return;

    int l = apply_deadzone(left_pwm_);
    int r = apply_deadzone(right_pwm_);

    // If stopped, send once
    if (l == 0 && r == 0) {
      if (!sent_stop_) {
        write_cmd(l, r);
        sent_stop_ = true;
      }
      return;
    }

    // Moving → keep streaming
    write_cmd(l, r);
    sent_stop_ = false;
  }

  void write_cmd(int l, int r)
  {
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer),
                       "L:%d R:%d\n", l, r);
    
    ssize_t written = write(serial_fd_, buffer, len);
    if (written < 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Serial write failed: %s", strerror(errno));
    } else if (written != len) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Partial write: %zd/%d bytes", written, len);
    }
    
    tcdrain(serial_fd_);
    
    // Debug output (throttled to avoid spam)
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 500,
                         "Sent: L:%d R:%d", l, r);
  }

  /* ---------------- ROS ---------------- */

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* ---------------- State ---------------- */

  int left_pwm_  = 0;
  int right_pwm_ = 0;

  bool left_ready_  = false;
  bool right_ready_ = false;
  bool sent_stop_   = false;

  int serial_fd_ = -1;
};

/* ---------------- main ---------------- */

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosSerialNode>());
  rclcpp::shutdown();
  return 0;
}
