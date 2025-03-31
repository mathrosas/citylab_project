#include "robot_patrol/srv/get_direction.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using GetDirection = robot_patrol::srv::GetDirection;

class ServiceTester : public rclcpp::Node {
public:
  ServiceTester() : Node("service_tester") {
    // Create client to the direction service
    client_ = this->create_client<GetDirection>("direction_service");

    // Subscribe to LaserScan data
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ServiceTester::scan_callback, this, _1));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Client<GetDirection>::SharedPtr client_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Waiting for direction_service...");
      return;
    }

    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *msg;

    auto future = client_->async_send_request(
        request, [this](rclcpp::Client<GetDirection>::SharedFuture result) {
          RCLCPP_INFO(this->get_logger(), "Direction: %s",
                      result.get()->direction.c_str());
        });
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServiceTester>());
  rclcpp::shutdown();
  return 0;
}
