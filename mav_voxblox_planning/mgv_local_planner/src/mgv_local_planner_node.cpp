#include "mgv_local_planner/mgv_local_planner.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mgv_local_planner");
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  FLAGS_alsologtostderr = true;

  mgv_planning::MgvLocalPlanner node(nh, nh_private);
  ROS_INFO("Initialized Mgv Local Planner node.");

  ros::spin();
  return 0;
}