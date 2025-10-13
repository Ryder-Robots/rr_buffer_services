#ifndef RR_IMAGE_SUBSCRIBER_HPP
#define RR_IMAGE_SUBSCRIBER_HPP

#include "rr_buffer_services/rr_abstact_subscriber.hpp"
#include "rr_interfaces/msg/buffer_response.hpp"


using namespace std;

namespace rrobot
{

/**
 * @class RrImageSubscriber
 * @brief configure subscription to image topic
 * 
 * configures subscriber for subscriptiosn to image to topic.
 */
class RrImageSubscriber : public RrABstractSubscriber
{
public:
  RrImageSubscriber() {}

  ~RrImageSubscriber() = default;

  /**
   * @fn callback
   * @brief called when images are available in raw image topic
   */
  void callback(const sensor_msgs::msg::Image::SharedPtr msg);

  std::string getTopicParam() override;
  std::string getQueueSzParam() override;
  std::string getTopicDefault() override;
  int getQueueSzDefault() override;

private:
  // Current image message.
  const sensor_msgs::msg::Image::SharedPtr msg_;
};

} // namespace rrobot

#endif // RR_IMAGE_SUBSCRIBER_HPP