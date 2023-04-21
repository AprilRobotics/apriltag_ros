#ifndef APRILTAG_ROS_TAG_BUNDLE_CALIBRATION_H
#define APRILTAG_ROS_TAG_BUNDLE_CALIBRATION_H

#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fstream>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include <vector>

namespace apriltag_ros
{
namespace detail
{

geometry_msgs::Pose averagePoses(const std::vector<geometry_msgs::Pose>& poses);
void writeToYaml(const std::unordered_map<int, geometry_msgs::Pose>& tag_poses_in_master_frame,
                 const std::unordered_map<int, double>& tag_size_map,
                 const std::string& tag_bundle_name,
                 std::ostream& os);
} // namespace detail

class TagBundleCalibrationNode
{
public:
  TagBundleCalibrationNode(int max_detections,
                           const std::string& config_file_path,
                           const std::string& tag_bundle_name,
                           int master_tag_id);

private:
  void tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

  ros::Subscriber tag_detection_sub_;
  int max_detections_;
  std::string config_file_path_;
  std::string tag_bundle_name_;
  int master_tag_id_;
  int received_detections_;
  std::unordered_map<int, geometry_msgs::Pose> tags_in_master_frame_;
  std::unordered_map<int, double> tag_size_map_;
};

} // namespace apriltag_ros

#endif // APRILTAG_ROS_TAG_BUNDLE_CALIBRATION_H
