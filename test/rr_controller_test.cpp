// have a look at https://medium.com/@junbs95/code-completion-and-debugging-for-ros2-in-vscode-a4ede900d979 for
// more information about debugging,  doesn't work unless on command prompt

// TODO: This test needs to be renamed to rr_state_maintainer_test

#include "rclcpp/rclcpp.hpp"
#include "rr_buffer_services/rr_batt_state_const.hpp"
#include "rr_buffer_services/rr_joystick_const.hpp"
#include "rr_buffer_services/rr_state_maintainer.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace rrobot;

class TestController : public testing::Test
{
protected:
  TestController()
  {
  }

  ~TestController() override
  {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  void SetUp() override
  {
  }

  void TearDown() override
  {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  rclcpp::Logger logger_ = rclcpp::get_logger("test_logger");
  RrStateMaintainer state_maintainer_ = RrStateMaintainer();
};


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}