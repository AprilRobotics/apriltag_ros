#include "apriltags2_ros/continuous_detector.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apriltags2_ros");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  apriltags2_ros::ContinuousDetector continuous_tag_detector(nh, pnh);

  ros::spin();
}
