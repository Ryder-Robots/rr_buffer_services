#include "rr_buffer_services/rr_image_subscriber.hpp"

using namespace rrobot;

void RrImageSubscriber::init()
{
  subscription_ =
    this->create_subscription<sensor_msgs::msg::Image>("topic", 10,  
        std::bind(&RrImageSubscriber::callback, this, std::placeholders::_1));
}


void RrImageSubscriber::callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    //TODO: Fill in the gaps
}