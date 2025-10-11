#ifndef RR_SUBSCRIBER_HPP
#define RR_SUBSCRIBER_HPP

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace rrobot
{
class RrSubscriber : public rclcpp::Node
{
public:
  RrSubscriber() : Node("rr_buffer_subscriber") {}
};
} // namespace rrobot

#endif