#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node"), direction_(0.0) {
    // Declare parameters
    this->declare_parameter("obstacle_threshold", 0.35);
    this->declare_parameter("linear_velocity", 0.1);
    this->declare_parameter("angular_gain", 0.5);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Patrol::laser_callback, this, _1));
    timer_ =
        this->create_wall_timer(100ms, std::bind(&Patrol::control_loop, this));
  }

private:
  float direction_;
  float obstacle_threshold_;
  float linear_velocity_;
  float angular_gain_;

  int angle_to_index(const sensor_msgs::msg::LaserScan::SharedPtr &scan,
                     float desired_angle_deg) const {
    float desired_angle_rad = desired_angle_deg * M_PI / 180.0;
    int index = static_cast<int>(std::round(
        (desired_angle_rad - scan->angle_min) / scan->angle_increment));
    return (index + scan->ranges.size()) % scan->ranges.size();
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_double();

    // Get indices for -15°, 0°, and 15° to check frontal proximity
    int front_idx_min15 = angle_to_index(msg, -15.0);
    int front_idx_0 = angle_to_index(msg, 0.0);
    int front_idx_15 = angle_to_index(msg, 15.0);

    float front_distance_min15 = msg->ranges[front_idx_min15];
    float front_distance_0 = msg->ranges[front_idx_0];
    float front_distance_15 = msg->ranges[front_idx_15];

    // Find the minimum distance among the three frontal points
    float front_distance;
    int front_idx;
    if (front_distance_min15 <= front_distance_0 &&
        front_distance_min15 <= front_distance_15) {
      front_distance = front_distance_min15;
      front_idx = front_idx_min15;
    } else if (front_distance_0 <= front_distance_15) {
      front_distance = front_distance_0;
      front_idx = front_idx_0;
    } else {
      front_distance = front_distance_15;
      front_idx = front_idx_15;
    }

    RCLCPP_DEBUG(this->get_logger(), "Front distance: %f", front_distance);

    if (front_distance <= obstacle_threshold_) {
      // Search for the safest direction in the front 180 degrees
      int start_idx = angle_to_index(msg, -90.0);
      int end_idx = angle_to_index(msg, 90.0);
      float max_distance = front_distance;
      int max_index = front_idx;

      // Handle potential index wrap-around
      if (start_idx <= end_idx) {
        for (int i = start_idx; i <= end_idx; i++) {
          update_max_distance(msg, i, max_distance, max_index);
        }
      } else {
        for (int i = start_idx; i < static_cast<int>(msg->ranges.size()); i++) {
          update_max_distance(msg, i, max_distance, max_index);
        }
        for (int i = 0; i <= end_idx; i++) {
          update_max_distance(msg, i, max_distance, max_index);
        }
      }

      // Calculate direction angle and clamp to [-π/2, π/2]
      direction_ = msg->angle_min + max_index * msg->angle_increment;
      if (direction_ > M_PI / 2)
        direction_ = M_PI / 2;
      else if (direction_ < -M_PI / 2)
        direction_ = -M_PI / 2;
    } else {
      direction_ = 0.0; // No obstacle, go straight
    }
  }

  void update_max_distance(const sensor_msgs::msg::LaserScan::SharedPtr &msg,
                           int index, float &max_distance,
                           int &max_index) const {
    if (std::isfinite(msg->ranges[index]) &&
        msg->ranges[index] > max_distance) {
      max_distance = msg->ranges[index];
      max_index = index;
    }
  }

  void control_loop() {
    linear_velocity_ = this->get_parameter("linear_velocity").as_double();
    angular_gain_ = this->get_parameter("angular_gain").as_double();

    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_velocity_;

    if (std::abs(direction_) > 0.0) {
      msg.linear.x *= 0.5; // Reduce speed while turning
      msg.angular.z = direction_ * angular_gain_;
    } else {
      msg.angular.z = 0.0;
    }

    publisher_->publish(msg);
  }

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