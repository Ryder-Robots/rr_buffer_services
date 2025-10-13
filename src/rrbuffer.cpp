#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rr_buffer_services/rr_subscriber_factory.hpp"

using namespace rrobot;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto message =  rr_interfaces::msg::BufferResponse();
  std::shared_mutex shared_mtx;

  std::shared_ptr<RrSubscriber> minSubscriber = std::make_shared<RrSubscriber>();
  // minSubscriber->init(message, shared_mtx);
  auto subscriberFactory = std::make_shared<RrSubscriberFactory>(minSubscriber);
  rclcpp::spin(minSubscriber);
  rclcpp::shutdown();
  return 0;
}