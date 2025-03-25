#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node"), direction_(0.0) {
    // Initialize publisher
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Initialize laser subscription
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Patrol::laser_callback, this, _1));

    // Initialize control timer (10Hz)
    timer_ =
        this->create_wall_timer(100ms, std::bind(&Patrol::control_loop, this));
  }

private:
  float direction_; // Stores safest direction angle

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Check front distance (35cm threshold)
    float front_distance = msg->ranges[360];
    RCLCPP_INFO(this->get_logger(), "front_dist: '%f'", msg->ranges[360]);

    if (front_distance < 0.35) {
      // Find safest direction in front 180 degrees
      int start_idx = 180;
      int end_idx = 540;

      float max_distance = 0.0;
      int max_index = msg->ranges.size() / 2; // Default to forward

      for (int i = start_idx; i < end_idx; i++) {
        if (std::isfinite(msg->ranges[i]) && msg->ranges[i] > max_distance) {
          max_distance = msg->ranges[i];
          max_index = i;
        }
      }

      // Calculate angle (-pi/2 to pi/2)
      direction_ = msg->angle_min + max_index * msg->angle_increment;

      // Normalize to [-pi/2, pi/2]
      if (direction_ > M_PI / 2)
        direction_ = M_PI / 2;
      else if (direction_ < -M_PI / 2)
        direction_ = -M_PI / 2;
    } else {
      direction_ = 0.0;
    }
  }

  void control_loop() {
    auto msg = geometry_msgs::msg::Twist();

    if (std::abs(direction_) > 0.0) {
      msg.linear.x = 0.0; // Stop forward motion
      msg.angular.z = direction_ / 2.0;
    } else {
      msg.linear.x = 0.1; // Move forward
      msg.angular.z = 0.0;
    }

    publisher_->publish(msg);
  }

  // Member variables
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}