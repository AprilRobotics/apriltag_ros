#ifndef APRILTAGS2_ROS_SINGLE_IMAGE_DETECTOR_H
#define APRILTAGS2_ROS_SINGLE_IMAGE_DETECTOR_H

#include "apriltags2_ros/common_functions.h"
#include <apriltags2_ros/AnalyzeSingleImage.h>

namespace apriltags2_ros
{

class SingleImageDetector
{
 private:
  TagDetector tag_detector_;
  ros::ServiceServer single_image_analysis_service_;

  ros::Publisher tag_detections_publisher_;
  
 public:
  SingleImageDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  // The function which provides the single image analysis service
  bool analyzeImage(apriltags2_ros::AnalyzeSingleImage::Request& request,
                     apriltags2_ros::AnalyzeSingleImage::Response& response);
};

} // namespace apriltags2_ros

#endif // APRILTAGS2_ROS_SINGLE_IMAGE_DETECTOR_H
