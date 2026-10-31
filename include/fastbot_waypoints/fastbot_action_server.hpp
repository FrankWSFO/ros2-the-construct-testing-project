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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class FastbotActionServer : public rclcpp::Node {
public:
  using Waypoint = fastbot_waypoints::action::WaypointAction;
  using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

  explicit FastbotActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::CallbackGroup::SharedPtr callback_group_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  geometry_msgs::msg::Point position_;
  float yaw_;
  geometry_msgs::msg::Point desired_position_;
  float desired_yaw_;

  float dist_precision_;
  float yaw_precision_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Waypoint::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleWaypoint> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle);

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle);

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  float convertQuaternionToEuler(const geometry_msgs::msg::Quaternion &msg_quat);
}; // class MyActionServer