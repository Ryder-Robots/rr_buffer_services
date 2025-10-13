#ifndef RR_SUBSCRIBER_FACTORY_HPP
#define RR_SUBSCRIBER_FACTORY_HPP

#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "rr_buffer_services/rr_subscriber.hpp"
#include "rr_buffer_services/rr_image_subscriber.hpp"

namespace rrobot
{
/**
 * @class RrSubscriberFactory
 * @brief factory for subscribers.
 * 
 * Provides various subscribers, this is done to simplify the logic within the subscriber. Each subscriber
 * will have its own type which needs to be treated as a separate type,  but the business logic is the same.
 * 
 * In order to simplify, the factory is used.
 * 
 * CAVEAT: This will need to be moved to a plugin.
 */
class RrSubscriberFactory
{
public:
  RrSubscriberFactory(std::shared_ptr<RrSubscriber> subscriber): subscriber_(subscriber)
  {
    init();
  }

  ~RrSubscriberFactory() = default;

  /**
     * @fn init
     * @brief registers the callbacks for each service that needs to be subscribed.
     */
  void init();


private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  std::shared_ptr<RrSubscriber> subscriber_;

  RrImageSubscriber image_subscriber_;
};
} // namespace rrobot

#endif