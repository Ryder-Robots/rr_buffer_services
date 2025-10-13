#include "rr_buffer_services/rr_controller.hpp"

using namespace rrobot;

void RrController::init()
{
}

void RrController::reset_response()
{
}

void RrController::callback()
{
}

void RrController::publish()
{
  std::shared_lock lock(mutex_);
}

void RrController::set_gps(const sensor_msgs::msg::NavSatFix gps)
{
  RCLCPP_DEBUG(this->get_logger(), "recieved GPS");
  std::unique_lock<std::shared_mutex> lock(mutex_);
  buffer_response_->feature_sets.has_gps = true;
  buffer_response_->gps = gps;
}

void RrController::set_joystick(const sensor_msgs::msg::Joy joy)
{
  RCLCPP_DEBUG(this->get_logger(), "recieved Joystick");
  std::unique_lock<std::shared_mutex> lock(mutex_);
  buffer_response_->feature_sets.has_joy = true;
  buffer_response_->joystick = joy;
}

void RrController::set_batt_state(const sensor_msgs::msg::BatteryState batt_state)
{
  RCLCPP_DEBUG(this->get_logger(), "recieved batt_state");
  std::unique_lock<std::shared_mutex> lock(mutex_);
  buffer_response_->feature_sets.has_batt_state = true;
  buffer_response_->batt_state = batt_state;
}

void RrController::set_img(const sensor_msgs::msg::Image img)
{
  RCLCPP_DEBUG(this->get_logger(), "recieved img");
  std::unique_lock<std::shared_mutex> lock(mutex_);
  buffer_response_->feature_sets.has_img = true;
  buffer_response_->img = img;
}

void RrController::set_imu(const sensor_msgs::msg::Imu imu)
{
  RCLCPP_DEBUG(this->get_logger(), "recieved IMU");
  std::unique_lock<std::shared_mutex> lock(mutex_);
  buffer_response_->feature_sets.has_img = true;
  buffer_response_->imu = imu;
}

void RrController::set_ranges(const std::list<sensor_msgs::msg::Range> ranges)
{
//   RCLCPP_DEBUG(this->get_logger(), "recieved ranges");
//   std::list<sensor_msgs::msg::Range> nranges(ranges);
//   std::unique_lock<std::shared_mutex> lock(mutex_);
//   buffer_response_->feature_sets.has_ranges = true;
//   buffer_response_->ranges = nranges;
}

