#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rr_buffer_services/rr_subscriber_factory.hpp"

using namespace rrobot;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<RrController> ctl = std::make_shared<RrController>();
  ctl->init();
  auto subscriberFactory = std::make_shared<RrSubscriberFactory>(ctl);
  rclcpp::spin(ctl);
  rclcpp::shutdown();
  return 0;
}