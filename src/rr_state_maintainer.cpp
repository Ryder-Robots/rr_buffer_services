#include "rr_buffer_services/rr_state_maintainer.hpp"

using namespace rrobot;

// gps setter/getter
void RrStateMaintainer::set_gps(const sensor_msgs::msg::NavSatFix gps)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  feature_set_->has_gps = true;
  gps_ =  std::make_shared<sensor_msgs::msg::NavSatFix>(gps);
}

bool RrStateMaintainer::has_gps()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return feature_set_->has_gps;
}

const sensor_msgs::msg::NavSatFix RrStateMaintainer::get_gps()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return *gps_;
}

void RrStateMaintainer::init()
{
}