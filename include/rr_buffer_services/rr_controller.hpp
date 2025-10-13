#ifndef RR_SUBSCRIBER_HPP
#define RR_SUBSCRIBER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rr_interfaces/msg/buffer_response.hpp"
#include <functional>
#include <list>
#include <memory>

namespace rrobot
{

/**
 * @class RrPublisher
 * @brief 
 */
class RrController : public rclcpp::Node
{
public:
  RrController() : Node("rr_buffer_controller")
  {}

  ~RrController() = default;

  /**
   * @fn init
   * @brief performs initlization, including creating the subscriber.
   */
  void init();

  /**
   * @fn set_gps
   * @brief set_gps
   * 
   * allows gps  subscriber to set current value.
   */
  void set_gps(const sensor_msgs::msg::NavSatFix);

  /**
   * @fn set_joystick
   * @brief allows joystick subscriber to set current value.
   */
  void set_joystick(const sensor_msgs::msg::Joy);

  /**
   * @fn set_batt_state
   * @brief allows battery_state subscriber to set current value.
   */
  void set_batt_state(const sensor_msgs::msg::BatteryState);

  /**
   * @fn set_img
   * @brief allows video stream subscriber to set current value.
   */
  void set_img(const sensor_msgs::msg::Image);

  /**
   * @fn set_imu
   * @brief allows IMU subscriber to set current value.
   */
  void set_imu(const sensor_msgs::msg::Imu);

  /**
   * @fn set_ranges
   * @brief allows Ultra-Sonics, radors and other distance sensor 
   * subscriber to set current value.
   */
  void set_ranges(const std::list<sensor_msgs::msg::Range>);

private:  
  /**
   * @fn reset_response 
   * @brief Creates new Response and new GUID
   */
  void reset_response();

  /**
   * @fn callback
   * @brief callbacks based on timer.
   */
  void callback();

  /**
   * @fn publish
   * @brief publishes to the buffer topic.
   * 
   */
  void publish();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rr_interfaces::msg::BufferResponse>::SharedPtr publisher_;
  size_t count_ = 0;

  // // Shared variables between subscribers, and publisher.
  rr_interfaces::msg::BufferResponse::SharedPtr buffer_response_;
  std::shared_mutex mutex_; // shared mutex to allow multiple readers or one writer
};
} // namespace rrobot

#endif