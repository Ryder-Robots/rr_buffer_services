#ifndef RR_SUBSCRIBER_HPP
#define RR_SUBSCRIBER_HPP

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rr_interfaces/msg/buffer_response.hpp"

namespace rrobot
{

/**
 * @class RrPublisher
 * @brief 
 */
class RrController : public rclcpp::Node
{
public:
  RrController() : 
    Node("rr_buffer_controller")
  {}

  ~RrController() = default;

  /**
   * @fn init
   * @brief performs initlization, including creating the subscriber.
   */
  void init(rr_interfaces::msg::BufferResponse::SharedPtr buffer_response_, std::shared_ptr<std::shared_mutex>  mutex_);

  /**
   * @fn next_response
   * @brief gets queued response, that will be sent to queue in next callback.
   */
  rr_interfaces::msg::BufferResponse::SharedPtr get_queued_response();

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
  std::shared_ptr<std::shared_mutex> mutex_;  // shared mutex to allow multiple readers or one writer
};
} // namespace rrobot

#endif