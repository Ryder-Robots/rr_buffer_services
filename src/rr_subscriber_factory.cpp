#include "rr_buffer_services/rr_subscriber_factory.hpp"

using namespace rrobot;

void RrSubscriberFactory::init()
{
  subscriber_->declare_parameter(image_subscriber_.getTopicParam(), image_subscriber_.getTopicDefault());
  subscriber_->declare_parameter(image_subscriber_.getQueueSzParam(), image_subscriber_.getQueueSzDefault());

  RCLCPP_INFO(subscriber_->get_logger(), "creating image subscriber");
  img_subscription_ =
    subscriber_->create_subscription<sensor_msgs::msg::Image>(
        subscriber_->get_parameter(image_subscriber_.getTopicParam()).as_string(), 
        subscriber_->get_parameter(image_subscriber_.getTopicParam()).as_int(),
      std::bind(&RrImageSubscriber::callback, image_subscriber_, std::placeholders::_1));
}

void RrSubscriberFactory::createSubscribers()
{
}