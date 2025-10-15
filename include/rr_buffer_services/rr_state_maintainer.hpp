#ifndef RR_STATE_MAINTAINER_HPP
#define RR_STATE_MAINTAINER_HPP

#include "rclcpp/logger.hpp"
#include "rr_interfaces/msg/buffer_response.hpp"
#include "rr_interfaces/msg/feature_set.hpp"
#include <list>
#include <mutex>
#include <shared_mutex>

namespace rrobot
{
/**
     * @class RrStateMaintainer
     * @brief Shared object that maintains state variables accross different subscribers, and publisers.
     * 
     */
class RrStateMaintainer
{
public:
  RrStateMaintainer(rclcpp::Logger logger) : logger_(logger) { init(); }
  ~RrStateMaintainer() = default;

  /**
     * @fn set_gps has_gps get_gps
     * @brief setters and getters relating to gps
     */
  void set_gps(const sensor_msgs::msg::NavSatFix gps);
  bool has_gps();
  const sensor_msgs::msg::NavSatFix get_gps();

  /**
     * @fn set_joystick has_joystick get_joystick
     * @brief setters and getters relating to joystick
     */
  void set_joystick(const sensor_msgs::msg::Joy joystick);
  bool has_joystick();
  const sensor_msgs::msg::Joy get_joystick();

  /**
     * @fn set_batt_state get_batt_state has_batt_state
     * @brief setters and getters relating to battery state
     */
  void set_batt_state(const sensor_msgs::msg::BatteryState);
  bool has_batt_state();
  const sensor_msgs::msg::BatteryState get_batt_state();

  /**
     * @fn set_image has_image get_image
     * @brief setters and getters relating to images (video stream)
     */
  void set_image(const sensor_msgs::msg::Image);
  bool has_image();
  const sensor_msgs::msg::Image get_image();

  /**
     * @fn set_imu has_imu get_imu
     * @brief setters and getters relating to imu (video stream)
     */
  void set_imu(const sensor_msgs::msg::Imu);
  bool has_imu();
  const sensor_msgs::msg::Imu get_imu();

  /**
     * @fn set_imu has_imu get_imu
     * @brief setters and getters relating to imu (video stream)
     */
  void set_ranges(const std::list<sensor_msgs::msg::Range>);
  bool has_ranges();
  const std::list<sensor_msgs::msg::Range> get_ranges();

  const rr_interfaces::msg::FeatureSet get_feature_set();


private:
  void init();

  // variables
  sensor_msgs::msg::NavSatFix gps_;
  sensor_msgs::msg::Joy joystick_;
  sensor_msgs::msg::BatteryState batt_state_;
  sensor_msgs::msg::Image img_;
  sensor_msgs::msg::Imu imu_;
  std::list<sensor_msgs::msg::Range> ranges_; // = std::list<sensor_msgs::msg::Range>();
  rr_interfaces::msg::FeatureSet feature_set_;

  // shared mutex to allow multiple readers or one writer
  std::shared_mutex mutex_;
  rclcpp::Logger logger_;
};
} // namespace rrobot

#endif