#include "rr_buffer_services/rr_image_subscriber.hpp"

using namespace rrobot;

/*
 * Bind callback to queue.
 */
void RrImageSubscriber::init()
{
  this->declare_parameter(RR_TOPIC_PARAM, RR_IMG_BUF_TOPIC_PARAM_DEFAULT);
  topic_ = this->get_parameter(RR_TOPIC_PARAM).as_string();
  subscription_ =
    this->create_subscription<sensor_msgs::msg::Image>(topic_, RR_IMG_BUF_TOPIC_QUEUE_SZ_DEFAULT,
      std::bind(&RrImageSubscriber::callback, this, std::placeholders::_1));
}


void RrImageSubscriber::callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  //TODO: Fill in the gaps
}