#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "fastbot_waypoints/action/waypoint_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "fastbot_waypoints/fastbot_action_server.hpp"



FastbotActionServer::FastbotActionServer(const rclcpp::NodeOptions &options)
    : Node("fastbot_action_server", options) {
  using namespace std::placeholders;

  this->action_server_ = rclcpp_action::create_server<Waypoint>(
      this, "fastbot_waypoint_as",
      std::bind(&FastbotActionServer::handle_goal, this, _1, _2),
      std::bind(&FastbotActionServer::handle_cancel, this, _1),
      std::bind(&FastbotActionServer::handle_accepted, this, _1));

  cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/fastbot/cmd_vel", 10);

  rclcpp::SubscriptionOptions options_odom;
  options_odom.callback_group = callback_group_odom_;
  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/fastbot/odom", 10,
      std::bind(&FastbotActionServer::odom_callback, this, _1), options_odom);

  dist_precision_ = 0.05;
  yaw_precision_ = M_PI / 90;
}

rclcpp_action::GoalResponse
FastbotActionServer::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                 std::shared_ptr<const Waypoint::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request with position %f, %f",
              goal->position.x, goal->position.y);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FastbotActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FastbotActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&FastbotActionServer::execute, this, _1), goal_handle}
      .detach();
}

void FastbotActionServer::execute(
    const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Waypoint::Feedback>();
  auto &feedback_message = feedback->position;
  // std_msgs::msg::Float64 publisher_message;
  // distance = 0;
  // feedback_message = distance;
  // publisher_message.data = distance;
  auto result = std::make_shared<Waypoint::Result>();
  rclcpp::Rate loop_rate(5);
  bool success;
  success = true;

// To uncomment to make test fail:
//     // Check if goal is done
//   if (rclcpp::ok()) {
//     result->success = true;
//     goal_handle->succeed(result);
//     RCLCPP_INFO(this->get_logger(), "Goal succeeded");
//     return;
//   }

  desired_position_ = goal->position;
  desired_yaw_ = std::atan2(std::sin(desired_position_.y - position_.y),
                            std::cos(desired_position_.x - position_.x));
  float err_position_ = hypot(desired_position_.y - position_.y,
                              desired_position_.x - position_.x);
  float err_yaw_ =
      std::atan2(std::sin(desired_yaw_ - yaw_), std::cos(desired_yaw_ - yaw_));

  while (err_position_ > dist_precision_ && rclcpp::ok()) {

    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      success = false;
      return;
    }

    // Update variables
    desired_yaw_ = atan2(desired_position_.y - position_.y,
                         desired_position_.x - position_.x);
    err_position_ = hypot(desired_position_.y - position_.y,
                          desired_position_.x - position_.x);
    err_yaw_ = std::atan2(std::sin(desired_yaw_ - yaw_),
                          std::cos(desired_yaw_ - yaw_));
    RCLCPP_INFO(this->get_logger(), "Desired position: %f, %f",
                desired_position_.x, desired_position_.y);
    RCLCPP_INFO(this->get_logger(), "Current position: %f, %f", position_.x,
                position_.y);
    RCLCPP_INFO(this->get_logger(), "Current yaw: %f", yaw_);
    RCLCPP_INFO(this->get_logger(), "Desired yaw: %f", desired_yaw_);
    RCLCPP_INFO(this->get_logger(), "Error yaw: %f", err_yaw_);
    RCLCPP_INFO(this->get_logger(), "Error position: %f", err_position_);

    if (abs(err_yaw_) > yaw_precision_) {
      RCLCPP_INFO(this->get_logger(), "Fix yaw");
      feedback->state = "Fix yaw";
      geometry_msgs::msg::Twist twist;
      twist.angular.z = err_yaw_ > 0 ? 1.2 : -1.2;
      twist.linear.x = 0;
      cmd_vel_publisher_->publish(twist);
    } else {
      RCLCPP_INFO(this->get_logger(), "Go to point");
      feedback->state = "Go to point";
      geometry_msgs::msg::Twist twist;
      twist.linear.x = 0.15;
      twist.angular.z = 0.0;
      cmd_vel_publisher_->publish(twist);
    }

    //   // Send feedback on distance travelled so far and likewise publish to
    //   distance topic
    feedback_message = position_;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publish feedback");

    loop_rate.sleep();
  }

  //  Stop
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.angular.z = 0.0;
  cmd_vel_publisher_->publish(twist);

  // Check if goal is done
  if (rclcpp::ok()) {
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

void FastbotActionServer::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  position_ = msg->pose.pose.position;
  yaw_ = convertQuaternionToEuler(msg->pose.pose.orientation);
  // RCLCPP_INFO(this->get_logger(),
  //             "X: %.2f, Y: %.2f, Yaw: %.2f (rad)", position_.x,
  //             position_.y, yaw_);
}

float FastbotActionServer::convertQuaternionToEuler(
    const geometry_msgs::msg::Quaternion &msg_quat) {
  tf2::Quaternion tf2_quat;

  tf2::fromMsg(msg_quat, tf2_quat);

  tf2::Matrix3x3 m(tf2_quat);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

//   For some reason the raw yaw is producing yaws pi/2 more than they should be, so correct here
  return atan2(sin(yaw-M_PI_2), cos(yaw-M_PI_2));
}
