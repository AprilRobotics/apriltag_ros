#include "apriltag_ros/tag_bundle_calibration.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tag_bundle_calibration_node");
  ros::NodeHandle nh{"~"};

  // Read params
  int max_detections;
  std::string config_file_path;
  std::string tag_bundle_name;
  int master_tag_id;
  nh.param("max_detections", max_detections, 500);
  nh.param("config_file_path", config_file_path, std::string("/tmp/tag_bundle_config.yaml"));
  nh.param("tag_bundle_name", tag_bundle_name, std::string("tag_bundle"));
  nh.param("master_tag_id", master_tag_id, 0);

  apriltag_ros::TagBundleCalibrationNode node(
      max_detections, config_file_path, tag_bundle_name, master_tag_id);

  ros::spin();
  return 0;
}