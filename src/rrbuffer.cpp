#include "rr_buffer_services/rr_image_subscriber.hpp"

using namespace rrobot;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RrImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}