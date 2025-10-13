#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rr_buffer_services/rr_subscriber_factory.hpp"

using namespace rrobot;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  shared_ptr message =  rr_interfaces::msg::BufferResponse::SharedPtr();
  std::shared_ptr<std::shared_mutex> shared_mtx = std::make_shared<std::shared_mutex>();
  std::shared_ptr<RrSubscriber> minSubscriber = std::make_shared<RrSubscriber>();
  minSubscriber->init(message, shared_mtx);
  auto subscriberFactory = std::make_shared<RrSubscriberFactory>(minSubscriber);
  rclcpp::spin(minSubscriber);
  rclcpp::shutdown();
  return 0;
}