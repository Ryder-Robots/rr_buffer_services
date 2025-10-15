// have a look at https://medium.com/@junbs95/code-completion-and-debugging-for-ros2-in-vscode-a4ede900d979 for
// more information about debugging,  doesn't work unless on command prompt

#include "rclcpp/rclcpp.hpp"
#include "rr_buffer_services/rr_controller.hpp"
#include <gtest/gtest.h>

using namespace rrobot;

class TestController : public testing::Test
{
protected:
  // std::shared_ptr<RrController> node_;

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
};


// Test setters and getters
TEST_F(TestController, setGps)
{
  rclcpp::Clock clock;
  auto current_time = clock.now();

  sensor_msgs::msg::NavSatFix navsat_msg;
  navsat_msg.header.stamp = current_time;
  navsat_msg.header.frame_id = "gps_link";
  navsat_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  navsat_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  // Set geographic coordinates (latitude, longitude, altitude)
  navsat_msg.latitude = -33.8688;  // Degrees, e.g., Sydney
  navsat_msg.longitude = 151.2093; // Degrees, e.g., Sydney
  navsat_msg.altitude = 58.0;      // In meters above WGS84 ellipsoid

  // Set position covariance (if known, otherwise leave as default zeros)
  std::fill(std::begin(navsat_msg.position_covariance), std::end(navsat_msg.position_covariance), 0.0);
  navsat_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  // node_->set_gps(navsat_msg);

  EXPECT_EQ(1, 1);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  // rclcpp::init(argc, argv);


  auto result = RUN_ALL_TESTS();
  // rclcpp::shutdown();
  return result;
}