#include "apriltags2_ros/single_image_detector.h"

#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Header.h>

namespace apriltags2_ros
{

SingleImageDetector::SingleImageDetector (ros::NodeHandle& nh,
                                          ros::NodeHandle& pnh) :
    tag_detector_(pnh)
{
  // Advertise the single image analysis service
  single_image_analysis_service_ =
      nh.advertiseService("single_image_tag_detection",
                          &SingleImageDetector::analyzeImage, this);
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  ROS_INFO_STREAM("Namespace: " << ros::this_node::getNamespace().c_str());
  ROS_INFO_STREAM("Ready to do tag detection on single images");
}

bool SingleImageDetector::analyzeImage(
    apriltags2_ros::AnalyzeSingleImage::Request& request,
    apriltags2_ros::AnalyzeSingleImage::Response& response)
{

  ROS_INFO("[ Summoned to analyze image ]");
  ROS_INFO("Image load path: %s",
           request.full_path_where_to_get_image.c_str());
  ROS_INFO("Image save path: %s",
           request.full_path_where_to_save_image.c_str());

  // Read the image
  cv::Mat image = cv::imread(request.full_path_where_to_get_image,
                             cv::IMREAD_COLOR);
  if (image.data == NULL)
  {
    // Cannot read image
    ROS_ERROR_STREAM("Could not read image " <<
                     request.full_path_where_to_get_image.c_str());
    return false;
  }

  // Detect tags in the image
  cv_bridge::CvImagePtr loaded_image(new cv_bridge::CvImage(std_msgs::Header(),
                                                            "bgr8", image));
  loaded_image->header.frame_id = "camera";
  response.tag_detections =
      tag_detector_.detectTags(loaded_image,sensor_msgs::CameraInfoConstPtr(
          new sensor_msgs::CameraInfo(request.camera_info)));

  // Publish detected tags (AprilTagDetectionArray, basically an array of
  // geometry_msgs/PoseWithCovarianceStamped)
  tag_detections_publisher_.publish(response.tag_detections);

  // Save tag detections image
  tag_detector_.drawDetections(loaded_image);
  cv::imwrite(request.full_path_where_to_save_image, loaded_image->image);

  ROS_INFO("Done!\n");

  return true;
}

} // namespace apriltags2_ros
