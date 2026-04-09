
#include "legged_ros_hw/RosHW.h"

#include <legged_hw/LeggedHWLoop.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "legged_ros_hw");
  ros::NodeHandle nh;
  ros::NodeHandle robotHwNh("~");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  try
  {
    std::shared_ptr<legged::RosHW> rosHw = std::make_shared<legged::RosHW>();
    rosHw->init(nh, robotHwNh);

    legged::LeggedHWLoop controlLoop(nh, rosHw);

    ros::waitForShutdown();
  }
  catch (const ros::Exception &e)
  {
    ROS_FATAL_STREAM("Error in the hardware interface:\n"
                     << "\t" << e.what());
    return 1;
  }

  return 0;
}
