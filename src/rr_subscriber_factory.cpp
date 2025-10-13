#include "rr_buffer_services/rr_subscriber_factory.hpp"

using namespace rrobot;

/**
 *  Initilizes callbacks for subscribers for car.
 */
void RrSubscriberFactory::init()
{
  RCLCPP_INFO(subscriber_->get_logger(), "creating callback group(s)");
  auto cg = subscriber_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;

  RCLCPP_INFO(subscriber_->get_logger(), "getting image subscriber params");
  subscriber_->declare_parameter(image_subscriber_.getTopicParam(), image_subscriber_.getTopicDefault());
  subscriber_->declare_parameter(image_subscriber_.getQueueSzParam(), image_subscriber_.getQueueSzDefault());

  RCLCPP_INFO(subscriber_->get_logger(), "creating image subscriber");
  image_subscriber_.set_ctl_node(subscriber_);
  auto img_callback =  std::bind(&RrImageSubscriber::callback, image_subscriber_, std::placeholders::_1);
  auto img_topic_str = subscriber_->get_parameter(image_subscriber_.getTopicParam()).as_string();
  img_subscription_ = subscriber_->create_subscription<sensor_msgs::msg::Image>(img_topic_str, rclcpp::SensorDataQoS(), img_callback, options);

  RCLCPP_INFO(subscriber_->get_logger(), "completed initilization");
}
