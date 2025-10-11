#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rr_buffer_services/rr_subscriber_factory.hpp"

using namespace rrobot;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<RrSubscriber> minSubscriber = std::make_shared<RrSubscriber>();
  auto subscriberFactory = std::make_shared<RrSubscriberFactory>(minSubscriber);
  rclcpp::spin(minSubscriber);
  rclcpp::shutdown();
  return 0;
}