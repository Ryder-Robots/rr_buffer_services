#include "rr_buffer_services/rr_state_maintainer.hpp"

using namespace rrobot;

// gps setter/getter
void RrStateMaintainer::set_gps(const sensor_msgs::msg::NavSatFix gps)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  feature_set_.has_gps = true;
  gps_ = gps;
}

bool RrStateMaintainer::has_gps()
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return feature_set_.has_gps;
}

const sensor_msgs::msg::NavSatFix RrStateMaintainer::get_gps()
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return gps_;
}

// default all feature sets to false to start with.
void RrStateMaintainer::init()
{
  feature_set_.has_batt_state = false;
  feature_set_.has_gps = false;
  feature_set_.has_img = false;
  feature_set_.has_imu = false;
  feature_set_.has_joy = false;
  feature_set_.has_ranges = false;
}