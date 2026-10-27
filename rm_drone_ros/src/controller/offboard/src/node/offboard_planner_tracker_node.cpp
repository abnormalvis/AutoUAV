#include "offboard_planner_tracker.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "offboard_planner_tracker");
  ros::NodeHandle nh("~");
  offboard::PlannerTracker tracker(nh);
  tracker.run();
  return 0;
}