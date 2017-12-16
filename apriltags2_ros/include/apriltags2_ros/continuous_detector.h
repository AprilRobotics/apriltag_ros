#ifndef APRILTAGS2_ROS_CONTINUOUS_DETECTOR_H
#define APRILTAGS2_ROS_CONTINUOUS_DETECTOR_H

#include "apriltags2_ros/common_functions.h"

namespace apriltags2_ros
{

class ContinuousDetector
{
 public:
  ContinuousDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  void imageCallback(const sensor_msgs::ImageConstPtr& image_rect,
                     const sensor_msgs::CameraInfoConstPtr& camera_info);

 private:
  TagDetector tag_detector_;
  bool draw_tag_detections_image_;
  cv_bridge::CvImagePtr cv_image_;

  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_image_subscriber_;
  image_transport::Publisher tag_detections_image_publisher_;
  ros::Publisher tag_detections_publisher_;
};

} // namespace apriltags2_ros

#endif // APRILTAGS2_ROS_CONTINUOUS_DETECTOR_H
