#ifndef RR_IMAGE_BUFFER_ACTION_HPP
#define RR_IMAGE_BUFFER_ACTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rr_interfaces/action/image_action.hpp"
#include <functional>
#include <memory>
#include <thread>

namespace rr_buffer_actions
{
class RrImageBufferAction : public rclcpp::Node
{
public:
  using ImageAction = rr_buffer_actions::RrImageBufferAction;
  using GoalHandleImageAction = rclcpp_action::ServerGoalHandle<ImageAction>;

  void
  init();

  explicit RrImageBufferAction(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("rr_buffer_services", options)
  {
    init();
  }

private:
  rclcpp_action::Server<ImageAction>::SharedPtr action_server_;
  void execute(const std::shared_ptr<GoalHandleImageAction> goal_handle);
};
} // namespace rr_buffer_actions


// TODO: check where this should be called.
RCLCPP_COMPONENTS_REGISTER_NODE(rr_buffer_actions::RrImageBufferAction)

#endif // RR_IMAGE_BUFFER_ACTION_HPP
