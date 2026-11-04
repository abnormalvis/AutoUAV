#include <gtest/gtest.h>

// catkin_add_gtest links against the gtest library but not gtest_main, and this
// workspace forces gtest to build from source, which provides no gtest_main
// target. Supply the entry point here.
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
