#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

#define PI 3.14159

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

  int angle_to_index(const sensor_msgs::msg::LaserScan::SharedPtr &scan,
                     float desired_angle_deg) {
    float desired_angle_rad = desired_angle_deg * PI / 180.0;
    int index = static_cast<int>(std::round(
        (desired_angle_rad - scan->angle_min) / scan->angle_increment));
    return ((index + scan->ranges.size()) % scan->ranges.size());
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int start_idx = angle_to_index(msg, -90.0);
    int end_idx = angle_to_index(msg, 90.0);

    int front_idx_15 = angle_to_index(msg, -15.0);
    int front_idx_0 = angle_to_index(msg, 0.0);
    int front_idx_51 = angle_to_index(msg, 15.0);

    float front_distance_15 = msg->ranges[front_idx_15];
    float front_distance_0 = msg->ranges[front_idx_0];
    float front_distance_51 = msg->ranges[front_idx_51];

    int front_idx;
    float front_distance;

    if (front_distance_15 <= front_distance_0 &&
        front_distance_15 <= front_distance_51) {
      front_distance = front_distance_15;
      front_idx = front_idx_15;
    } else if (front_distance_0 <= front_distance_51) {
      front_distance = front_distance_0;
      front_idx = front_idx_0;
    } else {
      front_distance = front_distance_51;
      front_idx = front_idx_51;
    }
    RCLCPP_INFO(this->get_logger(), "front_dist: '%f'", front_distance);

    // Check front distance (35cm threshold)
    if (front_distance <= 0.35) {
      // Find safest direction in front 180 degrees
      float max_distance = front_distance;
      int max_index = front_idx; // Default to forward

      for (int i = start_idx; i <= end_idx; i++) {
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
      RCLCPP_INFO(this->get_logger(), "direction_: '%f'", direction_);
    } else {
      direction_ = 0.0;
    }
  }

  void control_loop() {
    auto msg = geometry_msgs::msg::Twist();

    msg.linear.x = 0.1; // Move forward
    if (std::abs(direction_) > 0.0) {
      msg.angular.z = direction_ / 2.0;
    } else {
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