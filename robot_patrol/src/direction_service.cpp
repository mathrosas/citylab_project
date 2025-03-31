#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

using GetDirection = robot_patrol::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service_node") {
    srv_ = create_service<GetDirection>(
        "direction_service",
        std::bind(&DirectionService::direction_callback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Direction service ready.");
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;

  int angle_to_index(const sensor_msgs::msg::LaserScan &scan,
                     float angle_deg) const {
    float angle_rad = angle_deg * M_PI / 180.0;
    int index = static_cast<int>(
        std::round((angle_rad - scan.angle_min) / scan.angle_increment));
    return std::clamp(index, 0, static_cast<int>(scan.ranges.size()) - 1);
  }

  float sum_range(const std::vector<float> &ranges, int start, int end) {
    float sum = 0.0;
    for (int i = start; i < end; ++i) {
      if (i >= 0 && i < static_cast<int>(ranges.size())) {
        float r = ranges[i];
        if (std::isfinite(r) && r > 0.05) {
          sum += r;
        }
      }
    }
    return sum;
  }

  void
  direction_callback(const std::shared_ptr<GetDirection::Request> request,
                     const std::shared_ptr<GetDirection::Response> response) {
    const auto &scan = request->laser_data;
    const auto &ranges = scan.ranges;

    int idx_right_start = angle_to_index(scan, -90);
    int idx_right_end = angle_to_index(scan, -30);
    int idx_front_start = angle_to_index(scan, -30);
    int idx_front_end = angle_to_index(scan, 30);
    int idx_left_start = angle_to_index(scan, 30);
    int idx_left_end = angle_to_index(scan, 90);

    float total_right = sum_range(ranges, idx_right_start, idx_right_end);
    float total_front = sum_range(ranges, idx_front_start, idx_front_end);
    float total_left = sum_range(ranges, idx_left_start, idx_left_end);

    RCLCPP_INFO(this->get_logger(), "L: %.2f F: %.2f R: %.2f", total_left,
                total_front, total_right);

    // Obstacle threshold: only go forward if the front is clear
    float min_front_range = std::numeric_limits<float>::infinity();
    for (int i = idx_front_start; i < idx_front_end; ++i) {
      float r = ranges[i];
      if (std::isfinite(r) && r > 0.05) {
        min_front_range = std::min(min_front_range, r);
      }
    }

    if (min_front_range > 0.35) {
      response->direction = "forward";
    } else {
      response->direction = (total_left > total_right) ? "left" : "right";
    }

    RCLCPP_INFO(this->get_logger(), "Service response: %s",
                response->direction.c_str());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}
