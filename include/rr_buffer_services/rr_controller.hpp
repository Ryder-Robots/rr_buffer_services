#ifndef RR_SUBSCRIBER_HPP
#define RR_SUBSCRIBER_HPP

#include "rmw/rmw.h"
#include "rclcpp/rclcpp.hpp"
#include "rr_common_base/rr_state_maintainer.hpp"
#include "rr_common_base/rr_buf_factory.hpp"
#include "rr_interfaces/msg/buffer_response.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <functional>
#include <list>
#include <memory>
#include <pluginlib/class_loader.hpp>

namespace rr_buffer_services
{

// plugins to load during initilization.
#define RR_COMMON_BASE "rr_common_base"
#define RR_STATE_MAINTAINER "rrobot::RrStateMaintainer"
#define RR_STATE_MAINTAINER_CLASS rrobot::RrStateMaintainer
#define RR_DEFAULT_STATE_MAINTAINER "rr_common_plugins::RrStateMaintainerImpl"
#define RR_STATE_BUF_FACTORY "rrobot::RrBufFactory"
#define RR_STATE_BUF_FACTORY_CLASS rrobot::RrBufFactory
#define RR_DEFAULT_ROBOT_TYPE "rr_common_plugins::RrBufferFactorySimple"

// CLI arguments
#define RR_CLI_ROBOT_TYPE "robot-type"

// Standard topic (follow message structure for this.)
#define RR_BUF_TOPIC  "rr_interfaces/msg/buffer_response"

// TODO: make the following tunable.
#define RR_TOPIC_TICK 250
#define RR_QUEUE_LIMIT 10

/**
 * @class RrController
 * @brief coordinates subscriptions and publishing services for buffer
 * 
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

private:
  /**
   * @fn callback
   * @brief callbacks based on timer.
   * 
   * invokes publisher service to publish reponse message.
   */
  void callback();

  // sent timer that will be controlled within this executor.
  rclcpp::TimerBase::SharedPtr timer_;

  // callback groups
  rclcpp::CallbackGroup::SharedPtr publish_group_;
  rclcpp::CallbackGroup::SharedPtr sub_img_group_;
  rclcpp::CallbackGroup::SharedPtr sub_sensor_group_;

  rclcpp::Publisher<rr_interfaces::msg::BufferResponse>::SharedPtr publisher_;

  // updated for each request taht is sent
  size_t count_ = 0;

  std::shared_ptr<RR_STATE_MAINTAINER_CLASS> state_;
  std::shared_ptr<RR_STATE_BUF_FACTORY_CLASS> factory_;

  // internal initialization methods
  void init_state();
  void init_factory();

  // used for debugging
  std::string uuid_to_string(const unique_identifier_msgs::msg::UUID& uuid_msg);
};
} // namespace rr_buffer_services

#endif