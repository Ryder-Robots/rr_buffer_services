#include "rr_buffer_services/rr_subscriber_factory.hpp"

using namespace rrobot;

/**
 *  Initilizes callbacks for robot.
 * 
 * TODO: This should be moved to a specific package, and use a generic interface. It will be specific for each robot.
 */
void RrSubscriberFactory::init()
{
  RCLCPP_INFO(cnt_->get_logger(), "creating callback group(s)");
  auto cg = cnt_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions options;

  RCLCPP_INFO(cnt_->get_logger(), "getting image subscriber params");
  cnt_->declare_parameter(image_subscriber_.getTopicParam(), image_subscriber_.getTopicDefault());
  cnt_->declare_parameter(image_subscriber_.getQueueSzParam(), image_subscriber_.getQueueSzDefault());

  RCLCPP_INFO(cnt_->get_logger(), "creating image subscriber");
  image_subscriber_.set_ctl_node(cnt_);
  auto img_callback =  std::bind(&RrImageSubscriber::callback, image_subscriber_, std::placeholders::_1);
  auto img_topic_str = cnt_->get_parameter(image_subscriber_.getTopicParam()).as_string();
  img_subscription_ = cnt_->create_subscription<sensor_msgs::msg::Image>(img_topic_str, rclcpp::SensorDataQoS(), img_callback, options);

  RCLCPP_INFO(cnt_->get_logger(), "completed initilization");
}
