#include "rr_buffer_services/rr_image_subscriber.hpp"

using namespace rrobot;

std::string RrImageSubscriber::getTopicParam() {
  return "rr_image_topic";
}

std::string RrImageSubscriber::getQueueSzParam() {
  return "rr_queue_sz";
}


std::string RrImageSubscriber::getTopicDefault() {
  return "/camera/image_raw";
}


int RrImageSubscriber::getQueueSzDefault() {
  return 10;
}

void RrImageSubscriber::callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  //TODO: Fill in the gaps
}

void  RrImageSubscriber::set_ctl_node(std::shared_ptr<RrSubscriber>  ctl) {

}