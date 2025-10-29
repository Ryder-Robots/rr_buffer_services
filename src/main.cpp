#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rr_buffer_services/rr_controller.hpp"

using namespace rr_buffer_services;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RrController>();
  node->init();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}