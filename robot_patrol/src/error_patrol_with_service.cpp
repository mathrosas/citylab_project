#include "custom_interfaces/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

class PatrolWithService : public rclcpp::Node {
public:
  PatrolWithService() : Node("patrol_with_service_node") {
    declare_parameter("obstacle_threshold", 0.35);
    declare_parameter("linear_velocity", 0.1);
    declare_parameter("angular_velocity", 0.5);
    declare_parameter("stop_velocity", 0.0);

    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&PatrolWithService::laser_callback, this, _1));

    client_ = create_client<custom_interfaces::srv::GetDirection>(
        "direction_service");

    timer_ = create_wall_timer(
        100ms, std::bind(&PatrolWithService::control_loop, this));
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_scan_ = msg;
  }

  int angle_to_index(const sensor_msgs::msg::LaserScan::SharedPtr &scan,
                     float desired_angle_deg) const {
    float desired_angle_rad = desired_angle_deg * M_PI / 180.0;
    int index = std::round((desired_angle_rad - scan->angle_min) /
                           scan->angle_increment);
    return std::clamp(index, 0, static_cast<int>(scan->ranges.size()) - 1);
  }

  void control_loop() {
    if (!last_scan_)
      return;

    std::lock_guard<std::mutex> lock(mutex_);
    int front_index = angle_to_index(last_scan_, 0);
    float front_distance = last_scan_->ranges[front_index];

    auto msg = geometry_msgs::msg::Twist();
    float threshold = get_parameter("obstacle_threshold").as_double();
    float linear_velocity = get_parameter("linear_velocity").as_double();
    float angular_velocity = get_parameter("angular_velocity").as_double();
    float stop_velocity = get_parameter("stop_velocity").as_double();

    msg.linear.x = linear_velocity;
    msg.angular.z = stop_velocity;

    if (front_distance < threshold) {
      // Only call service if not already waiting
      if (!waiting_for_service_response_) {
        call_direction_service();
        waiting_for_service_response_ = true;
      }
    }

    if (direction_ == "forward") {
      msg.linear.x = linear_velocity;
      msg.angular.z = stop_velocity;
    } else if (direction_ == "left") {
      msg.linear.x = linear_velocity;
      msg.angular.z = angular_velocity;
    } else if (direction_ == "right") {
      msg.linear.x = linear_velocity;
      msg.angular.z = -angular_velocity;
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown direction: '%s'",
                  direction_.c_str());
    }
    RCLCPP_INFO(this->get_logger(),
                "Publishing velocity: linear=%.2f angular=%.2f direction=%s",
                msg.linear.x, msg.angular.z, direction_.c_str());

    publisher_->publish(msg);
  }

  void call_direction_service() {
    if (!client_->wait_for_service(100ms)) {
      RCLCPP_WARN(this->get_logger(), "direction_service not available");
      return;
    }

    auto request =
        std::make_shared<custom_interfaces::srv::GetDirection::Request>();
    request->laser_data = *last_scan_;
    auto future_result = client_->async_send_request(
        request, std::bind(&Patrol::publishVel, this, _1));

    client_->async_send_request(
        request,
        [this](
            rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedFuture
                result) {
          std::lock_guard<std::mutex> lock(mutex_);
          direction_ = result.get()->direction;
          waiting_for_service_response_ = false;
          RCLCPP_INFO(this->get_logger(), "Service responded: %s",
                      direction_.c_str());
        });
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  std::string direction_ = "forward";
  bool waiting_for_service_response_ = false;
  std::mutex mutex_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolWithService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}