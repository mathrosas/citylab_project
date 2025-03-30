#include "custom_interfaces/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using GetDirection = custom_interfaces::srv::GetDirection;
using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;
using namespace std::chrono_literals;
using std::placeholders::_1;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    this->declare_parameter("linear_velocity", 0.1);
    this->declare_parameter("angular_velocity", 0.5);
    this->declare_parameter("stop_velocity", 0.0);

    publisher_ = create_publisher<Twist>("cmd_vel", 10);

    subscription_ = create_subscription<LaserScan>(
        "scan", 10, std::bind(&Patrol::laser_callback, this, _1));

    client_ = create_client<GetDirection>("direction_service");

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service.");
    }
  }

private:
  void laser_callback(const LaserScan::SharedPtr msg) {
    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *msg;
    client_->async_send_request(request,
                                std::bind(&Patrol::handle_response, this, _1));
  }

  void
  handle_response(const rclcpp::Client<GetDirection>::SharedFuture future) {
    double linear_velocity = this->get_parameter("linear_velocity").as_double();
    double angular_velocity =
        this->get_parameter("angular_velocity").as_double();
    double stop_velocity = this->get_parameter("stop_velocity").as_double();

    auto msg = Twist();
    auto response = future.get();
    std::string direction = response->direction;

    RCLCPP_INFO(this->get_logger(), "Got direction: %s", direction.c_str());

    if (direction == "forward") {
      msg.linear.x = linear_velocity;
      msg.angular.z = stop_velocity;
    } else if (direction == "left") {
      msg.linear.x = linear_velocity;
      msg.angular.z = angular_velocity;
    } else if (direction == "right") {
      msg.linear.x = linear_velocity;
      msg.angular.z = -angular_velocity;
    } else {
      msg.linear.x = stop_velocity;
      msg.angular.z = stop_velocity;
    }

    publisher_->publish(msg);
  }

  rclcpp::Publisher<Twist>::SharedPtr publisher_;
  rclcpp::Subscription<LaserScan>::SharedPtr subscription_;
  rclcpp::Client<GetDirection>::SharedPtr client_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}