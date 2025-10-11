#ifndef RR_IMAGE_SUBSCRIBER_HPP
#define RR_IMAGE_SUBSCRIBER_HPP

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "rr_buffer_services/rr_subscriber.hpp"

/*
 * Defaults to use with RrImageSubscriber class
 */
#define RR_IMG_BUF_NODE_NAME "rr_image_subscriber"
#define RR_IMG_BUF_TOPIC_PARAM_DEFAULT "/camera/image_raw"
#define RR_IMG_BUF_TOPIC_QUEUE_SZ_DEFAULT 10

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
  RrImageSubscriber() : 
    Node(RR_IMG_BUF_NODE_NAME),
    topic_(RR_IMG_BUF_TOPIC_PARAM_DEFAULT),
    topic_queue_sz_(RR_IMG_BUF_TOPIC_QUEUE_SZ_DEFAULT)
  {
    init();
  }

  ~RrImageSubscriber() = default;

  /**
   * @fn init
   * @brief initilize subscriber
   * 
   * performed during initlization of the image subscriber.
   */
  void init();

  /**
   * @fn callback
   * @brief called when images are available in raw image topic
   */
  void callback(const sensor_msgs::msg::Image::SharedPtr msg);


private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  string topic_;
  int topic_queue_sz_;
};

} // namespace rrobot

#endif // RR_IMAGE_SUBSCRIBER_HPP