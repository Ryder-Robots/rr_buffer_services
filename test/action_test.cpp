#include <gtest/gtest.h>

TEST(rr_buffer_services, a_first_test)
{
  ASSERT_EQ(4, 2 + 2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}