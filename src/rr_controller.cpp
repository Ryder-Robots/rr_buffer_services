#include "rr_buffer_services/rr_controller.hpp"

using namespace rr_buffer_services;

void RrController::init_state()
{
  pluginlib::ClassLoader<RR_STATE_MAINTAINER_CLASS> poly_loader(RR_COMMON_BASE, RR_STATE_MAINTAINER);
  state_ = poly_loader.createSharedInstance(RR_DEFAULT_STATE_MAINTAINER);
}

void RrController::init_factory()
{
  this->declare_parameter<std::string>(RR_CLI_ROBOT_TYPE, RR_DEFAULT_ROBOT_TYPE);
  // load factory base class.
  pluginlib::ClassLoader<RR_STATE_BUF_FACTORY_CLASS> poly_loader(RR_COMMON_BASE, RR_STATE_BUF_FACTORY);

  RCLCPP_INFO(this->get_logger(), "loading robot '%s'", this->get_parameter(RR_CLI_ROBOT_TYPE).as_string().c_str());
  factory_ = poly_loader.createSharedInstance(this->get_parameter(RR_CLI_ROBOT_TYPE).as_string());

  RCLCPP_INFO(this->get_logger(), "initializing robot '%s'", this->get_parameter(RR_CLI_ROBOT_TYPE).as_string().c_str());
  factory_->initialize(shared_from_this(), state_);
}

void RrController::init()
{
  try {
    RCLCPP_INFO(this->get_logger(), "loading plugins");
    init_state();
    init_factory();
  }
  catch (pluginlib::PluginlibException &ex) {
    RCLCPP_FATAL(this->get_logger(), "could not load plugins, failed on the following: %s", ex.what());
  }
}


void RrController::callback()
{
}