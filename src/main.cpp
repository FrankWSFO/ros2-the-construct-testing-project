#include "fastbot_waypoints/fastbot_action_server.hpp"
#include <rclcpp/rclcpp.hpp>


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<FastbotActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}