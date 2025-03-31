#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "robot_patrol/action/go_to_pose.hpp"

using namespace std::chrono_literals;

class GoToPose : public rclcpp::Node {
public:
  using GoToPoseAction = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

  GoToPose() : Node("go_to_pose_action_server") {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<GoToPoseAction>(
        this, "/go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));

    vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  geometry_msgs::msg::Pose2D current_pos_;
  geometry_msgs::msg::Pose2D desired_pos_;
  bool odom_ready_ = false;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;

    // Convert quaternion to yaw manually (no tf2)
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    current_pos_.theta = std::atan2(siny_cosp, cosy_cosp);

    odom_ready_ = true;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &,
              std::shared_ptr<const GoToPoseAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal: x=%.2f y=%.2f theta=%.2f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Cancel request received.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    std::thread{std::bind(&GoToPose::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal...");
    rclcpp::Rate rate(10.0);
    auto feedback = std::make_shared<GoToPoseAction::Feedback>();
    auto result = std::make_shared<GoToPoseAction::Result>();

    const auto goal = goal_handle->get_goal();
    desired_pos_ = goal->goal_pos;

    while (!odom_ready_ && rclcpp::ok()) {
      rclcpp::sleep_for(100ms);
    }

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        vel_pub_->publish(geometry_msgs::msg::Twist()); // stop
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled.");
        return;
      }

      // Compute distance and angle to goal
      double dx = desired_pos_.x - current_pos_.x;
      double dy = desired_pos_.y - current_pos_.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      double angle_to_goal = std::atan2(dy, dx);
      double angle_error = angle_to_goal - current_pos_.theta;

      // Normalize angle error
      while (angle_error > M_PI)
        angle_error -= 2 * M_PI;
      while (angle_error < -M_PI)
        angle_error += 2 * M_PI;

      // Feedback
      feedback->current_pos = current_pos_;
      goal_handle->publish_feedback(feedback);

      geometry_msgs::msg::Twist cmd;

      if (distance > 0.05) {
        cmd.linear.x = 0.2;
        cmd.angular.z = 1.5 * angle_error;
      } else {
        // Final orientation adjustment
        double goal_theta_rad = desired_pos_.theta * M_PI / 180.0;
        double theta_error = goal_theta_rad - current_pos_.theta;

        while (theta_error > M_PI)
          theta_error -= 2 * M_PI;
        while (theta_error < -M_PI)
          theta_error += 2 * M_PI;

        if (std::abs(theta_error) > 0.05) {
          cmd.angular.z = 1.5 * theta_error;
        } else {
          vel_pub_->publish(geometry_msgs::msg::Twist()); // stop
          result->status = true;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal reached!");
          return;
        }
      }

      vel_pub_->publish(cmd);
      rate.sleep();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToPose>());
  rclcpp::shutdown();
  return 0;
}
