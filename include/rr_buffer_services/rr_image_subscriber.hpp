#ifndef RR_IMAGE_SUBSCRIBER_HPP
#define RR_IMAGE_SUBSCRIBER_HPP

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std;

namespace rrobot
{

/**
 * @class RrImageSubscriber
 * @brief subscribe to image topic
 * 
 * Subscribes to image topic and returns image when requested.
 */
class RrImageSubscriber : public rclcpp::Node
{
public:
  RrImageSubscriber() : Node("rr_image_subscriber")
  {
    init();
  }

  ~RrImageSubscriber() = default;

  void init();

  void callback(const sensor_msgs::msg::Image::SharedPtr msg);


private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

} // namespace rrobot

#endif // RR_IMAGE_SUBSCRIBER_HPP