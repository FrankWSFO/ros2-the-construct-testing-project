#include "gtest/gtest.h"
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

#include "fastbot_waypoints/fastbot_action_server.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;
using Waypoint = fastbot_waypoints::action::WaypointAction;
using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class WaypointActionTestFixture : public ::testing::Test {
public:
  WaypointActionTestFixture() : odom_detected_alive_(false) {

    fastbotActionServerNode_ = std::make_shared<FastbotActionServer>();

    actionClientNode_ = rclcpp::Node::make_shared("fastbot_action_client");
    actionClient_ = rclcpp_action::create_client<Waypoint>(
        actionClientNode_, "fastbot_waypoint_as");

    subscriberNode_ = rclcpp::Node::make_shared("odom_subscriber_node");

    // data_publisher_ = pubSubNode->create_publisher<std_msgs::msg::Float32>(
    //     "distance_to_vehicle_ahead", 10);

    odomSubscriber_ =
        subscriberNode_->create_subscription<nav_msgs::msg::Odometry>(
            "/fastbot/odom", 10,
            std::bind(&WaypointActionTestFixture::odom_callback, this, _1));

    exec_.add_node(fastbotActionServerNode_);
    exec_.add_node(actionClientNode_);
    exec_.add_node(subscriberNode_);
  }

protected:
  void waypointAccuracyTest(geometry_msgs::msg::Point waypoint_goal);

  void SetUp() override {
    ASSERT_TRUE(actionClient_->wait_for_action_server(5s));
    auto start = std::chrono::steady_clock::now();
    while (!odom_detected_alive_ &&
           std::chrono::steady_clock::now() - start < 5s) {
      exec_.spin_some();
    }
    ASSERT_TRUE(odom_detected_alive_);
  }

  void TearDown() override { actionClient_->async_cancel_all_goals(); }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    position_.x = msg->pose.pose.position.x;
    position_.y = msg->pose.pose.position.y;
    yaw_ = fastbotActionServerNode_->convertQuaternionToEuler(
        msg->pose.pose.orientation);

    odom_detected_alive_ = true;

    // callback_success_ = true;
  }
  std::shared_ptr<FastbotActionServer> fastbotActionServerNode_;
  rclcpp::Node::SharedPtr actionClientNode_;
  rclcpp::Node::SharedPtr subscriberNode_;
  rclcpp_action::Client<Waypoint>::SharedPtr actionClient_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscriber_;
  bool odom_detected_alive_;
  geometry_msgs::msg::Point position_;
  float yaw_;
  rclcpp::executors::MultiThreadedExecutor exec_;
};

void WaypointActionTestFixture::waypointAccuracyTest(
    geometry_msgs::msg::Point waypoint_goal) {
  float expected_yaw =
      atan2(waypoint_goal.y - position_.y, waypoint_goal.x - position_.x);
  Waypoint::Goal goal;
  goal.position = waypoint_goal;
  auto goal_future = actionClient_->async_send_goal(goal);
  EXPECT_EQ(exec_.spin_until_future_complete(goal_future, 5s),
            rclcpp::FutureReturnCode::SUCCESS);
  auto handle = goal_future.get();
  EXPECT_NE(handle, nullptr); // null means the goal was rejected

  auto result_future = actionClient_->async_get_result(handle);
  EXPECT_EQ(exec_.spin_until_future_complete(result_future, 60s),
            rclcpp::FutureReturnCode::SUCCESS);
  auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->success);

  EXPECT_NEAR(position_.x, waypoint_goal.x, 0.05);
  EXPECT_NEAR(position_.y, waypoint_goal.y, 0.05);
  EXPECT_NEAR(yaw_, expected_yaw, 0.5);
  //   std_msgs::msg::Float32 data_send;
  //   data_send.data = test_data;
  //   while (!callback_success_) {
  //     data_publisher_->publish(data_send);
  //     std::this_thread::sleep_for(1s);
  //     rclcpp::spin_some(pubSubNode);
  //     rclcpp::spin_some(distCtrlNode);
  //   }
  //   return received_data_;
}

TEST_F(WaypointActionTestFixture, SimpleTest30) {
  geometry_msgs::msg::Point waypoint_goal;
  waypoint_goal.x = 1.0;
  waypoint_goal.y = 1.0;
  //   EXPECT_FLOAT_EQ(0.2, waypointAccuracyTest(waypoint_goal));
  waypointAccuracyTest(waypoint_goal);
}