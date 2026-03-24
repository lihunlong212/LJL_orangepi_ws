#include <clocale>

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "activity_control_pkg/route_target_publisher.hpp"

int main(int argc, char ** argv)
{
  std::setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  std::vector<std::string> args;
  for (int i = 1; i < argc; ++i) {
    args.emplace_back(argv[i]);
  }
  options.arguments(args);

  auto route_node =
    std::make_shared<activity_control_pkg::RouteTargetPublisherNode>(options);

  auto test_node =
    std::make_shared<activity_control_pkg::RouteTestNode>(route_node, options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(route_node);
  executor.add_node(test_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}


