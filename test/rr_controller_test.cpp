#include "rr_buffer_services/rr_controller.hpp"
#include <gtest/gtest.h>

using namespace rrobot;

class TestController : public testing::Test
{
protected:
  TestController() : ctl_()
  {
  }

  ~TestController() override
  {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  void SetUp() override
  {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  void TearDown() override
  {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  RrController ctl_;
};

/**
 * Test goals within the controller.
 */


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}