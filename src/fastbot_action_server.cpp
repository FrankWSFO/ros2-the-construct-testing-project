#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "fastbot_waypoints/action/waypoint_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;
using namespace std;

class FastbotActionServer : public rclcpp::Node
{
public:
  using Waypoint = fastbot_waypoints::action::WaypointAction;
  using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

  explicit FastbotActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fastbot_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Waypoint>(
      this,
      "fastbot_waypoint_as",
      std::bind(&FastbotActionServer::handle_goal, this, _1, _2),
      std::bind(&FastbotActionServer::handle_cancel, this, _1),
      std::bind(&FastbotActionServer::handle_accepted, this, _1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/fastbot/cmd_vel", 10);

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&FastbotActionServer::odom_callback, this, _1));

  }

private:
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  geometry_msgs::msg::Point position_;
  float yaw_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Waypoint::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with position %f, %f", goal->position.x, goal->position.y);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleWaypoint> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FastbotActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypoint> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    // const auto goal = goal_handle->get_goal();
    // auto feedback = std::make_shared<Distance::Feedback>();
    // auto & feedback_message = feedback->current_dist;
    // std_msgs::msg::Float64 publisher_message;
    // distance = 0;
    // feedback_message = distance;
    // publisher_message.data = distance;
    auto result = std::make_shared<Waypoint::Result>();
    rclcpp::Rate loop_rate(25);

    // for (int i = 0; (i < goal->seconds) && rclcpp::ok(); ++i) {
    //   // Check if there is a cancel request
    //   if (goal_handle->is_canceling()) {
    //     result->status = false;
    //     goal_handle->canceled(result);
    //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
    //     return;
    //   }
    //   // Send feedback on distance travelled so far and likewise publish to distance topic
    //   feedback_message = distance;
    //   goal_handle->publish_feedback(feedback);
    //   RCLCPP_INFO(this->get_logger(), "Publish feedback");
    //   publisher_message.data = distance;
    //   publisher_->publish(publisher_message);

    //   loop_rate.sleep();
    // }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    position_ = msg->pose.pose.position;
    yaw_ = convertQuaternionToEuler(msg->pose.pose.orientation);
    RCLCPP_INFO(this->get_logger(), 
                "X: %.2f, Y: %.2f, Yaw: %.2f (rad)", position_.x, position_.y, yaw_);

  }

  float convertQuaternionToEuler(const geometry_msgs::msg::Quaternion& msg_quat)
    {
    tf2::Quaternion tf2_quat;

    tf2::fromMsg(msg_quat, tf2_quat);

    tf2::Matrix3x3 m(tf2_quat);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
  }
};  // class MyActionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<FastbotActionServer>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

// #! /usr/bin/env python
// import rospy
// import time
// import actionlib

// from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction
// from geometry_msgs.msg import Twist
// from std_msgs.msg import Empty
// from geometry_msgs.msg import Twist, Point
// from nav_msgs.msg import Odometry
// from tf import transformations
// import math

// class WaypointActionClass(object):

//     # create messages that are used to publish feedback/result
//     _feedback = WaypointActionFeedback()
//     _result = WaypointActionResult()

//     # topics
//     _pub_cmd_vel = None
//     _sub_odom = None

//     # go to point vars
//     # robot state variables
//     _position = Point()
//     _yaw = 0
//     # machine state
//     _state = 'idle'
//     # goal
//     _des_pos = Point()
//     # parameters
//     _yaw_precision = math.pi / 90 # +/- 2 degree allowed
//     _dist_precision = 0.05

//     def __init__(self):
//         # creates the action server
//         self._as = actionlib.SimpleActionServer("tortoisebot_as", WaypointActionAction, self.goal_callback, False)
//         self._as.start()

//         # define a loop rate
//         self._rate = rospy.Rate(25)

//         # topics
//         self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
//         self._sub_odom = rospy.Subscriber('/odom', Odometry, self._clbk_odom)
//         rospy.loginfo("Action server started")

//     def _clbk_odom(self, msg):
//         # position
//         self._position = msg.pose.pose.position

//         # yaw
//         quaternion = (
//             msg.pose.pose.orientation.x,
//             msg.pose.pose.orientation.y,
//             msg.pose.pose.orientation.z,
//             msg.pose.pose.orientation.w)
//         euler = transformations.euler_from_quaternion(quaternion)
//         self._yaw = euler[2]

//     def goal_callback(self, goal):
//         rospy.loginfo("goal %s received" % str(goal))

//         # helper variables
//         success = True

//         # define desired position and errors
//         self._des_pos = goal.position
//         # # To make error:
//         # self._des_pos.x = goal.position.x - 0.1
//         # self._des_pos.y = goal.position.y - 0.1
//         desired_yaw = math.atan2(self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
//         err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))
//         err_yaw = desired_yaw - self._yaw

//         # perform task
//         # while err_pos > self._dist_precision and math.fabs(err_yaw) > self._yaw_precision and success:
//         while err_pos > self._dist_precision and success:
//             # update vars
//             desired_yaw = math.atan2(self._des_pos.y - self._position.y, self._des_pos.x - self._position.x)
//             err_yaw = desired_yaw - self._yaw
//             err_pos = math.sqrt(pow(self._des_pos.y - self._position.y, 2) + pow(self._des_pos.x - self._position.x, 2))
//             # rospy.loginfo("Current Yaw: %s" % str(self._yaw))
//             # rospy.loginfo("Desired Yaw: %s" % str(desired_yaw))
//             rospy.loginfo("Error Yaw: %s" % str(err_yaw))
//             rospy.loginfo("Error Pos: %s" % str(err_pos))
//             # rospy.loginfo("Wrong error Yaw: %s" % str(wrong_err_yaw))
//             # logic goes here
//             if self._as.is_preempt_requested():
//                 # cancelled 
//                 rospy.loginfo("The goal has been cancelled/preempted")
//                 self._as.set_preempted()
//                 success = False
//             elif math.fabs(err_yaw) > self._yaw_precision:
//                 # fix yaw
//                 rospy.loginfo("fix yaw")
//                 self._state = 'fix yaw'
//                 twist_msg = Twist()
//                 twist_msg.angular.z = 0.65 if err_yaw > 0 else -0.65
//                 self._pub_cmd_vel.publish(twist_msg)
//             else:
//                 # go to point
//                 rospy.loginfo("go to point")
//                 self._state = 'go to point'
//                 twist_msg = Twist()
//                 twist_msg.linear.x = 0.6
//                 twist_msg.angular.z = 0
//                 # twist_msg.angular.z = 0.1 if err_yaw > 0 else -0.1
//                 self._pub_cmd_vel.publish(twist_msg)

//             # send feedback
//             self._feedback.position = self._position
//             self._feedback.state = self._state
//             self._as.publish_feedback(self._feedback)

//             # loop rate
//             self._rate.sleep()

//         # stop
//         twist_msg = Twist()
//         twist_msg.linear.x = 0
//         twist_msg.angular.z = 0
//         self._pub_cmd_vel.publish(twist_msg)

//         # # TROLLING To GIVE ERRORS 
//         # troll_count = 0
//         # while troll_count < 75:
//         #     twist_msg = Twist()
//         #     twist_msg.linear.x = 0.0
//         #     twist_msg.angular.z = 0.2
//         #     self._pub_cmd_vel.publish(twist_msg)
//         #     self._rate.sleep()
//         #     rospy.loginfo("Trolling")
//         #     troll_count+=1
        
//         # # stop
//         # twist_msg = Twist()
//         # twist_msg.linear.x = 0
//         # twist_msg.angular.z = 0
//         # self._pub_cmd_vel.publish(twist_msg)

//         # # END TROLL

//         # return success
//         if success:
//             self._result.success = True
//             self._as.set_succeeded(self._result)

// if __name__ == '__main__':
//     rospy.init_node('tortoisebot_as')
//     WaypointActionClass()
//     rospy.spin()