#include <apriltags2_ros/common_header.h>
#include <apriltags2_ros/continuous_detector.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "apriltags2_ros");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~"); // Private node handle for accessing ROS Parameter Server parameters in the namespace of the node

  ROS_INFO("Namespace: %s", pnh.getNamespace().c_str());
  
  apriltags2_ros::ContinuousDetector continuous_tag_detector(nh, pnh);

  ros::spin();
}
