#ifndef APRILTAGS2_ROS_SINGLE_IMAGE_DETECTOR_H
#define APRILTAGS2_ROS_SINGLE_IMAGE_DETECTOR_H

#include <apriltags2_ros/common_header.h>
#include <apriltags2_ros/AnalyzeSingleImage.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Header.h>

namespace apriltags2_ros {

class SingleImageDetector {
 public:

  SingleImageDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  // The function which provides the single image analysis service
  bool analyze_image(apriltags2_ros::AnalyzeSingleImage::Request& request,
                     apriltags2_ros::AnalyzeSingleImage::Response& response);

 private:

  TagDetector tag_detector_;
  ros::ServiceServer single_image_analysis_service_;

  ros::Publisher tag_detections_publisher_;

};

}

#endif //APRILTAGS2_ROS_SINGLE_IMAGE_DETECTOR_H
