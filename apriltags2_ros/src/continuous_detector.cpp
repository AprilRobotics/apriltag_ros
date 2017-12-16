#include "apriltags2_ros/continuous_detector.h"

namespace apriltags2_ros
{

ContinuousDetector::ContinuousDetector (ros::NodeHandle& nh,
                                        ros::NodeHandle& pnh) :
    tag_detector_(pnh),
    draw_tag_detections_image_(
        getAprilTagOption<bool>(pnh, "publish_tag_detections_image", false)),
    it_(nh)
{
  camera_image_subscriber_ =
      it_.subscribeCamera("image_rect", 1,
                          &ContinuousDetector::imageCallback, this);
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_.advertise("tag_detections_image", 1);
  }
}

void ContinuousDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& image_rect,
    const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTags 2 on the iamge
  try
  {
    cv_image_ = cv_bridge::toCvCopy(image_rect,
                                    sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTags 2
  tag_detections_publisher_.publish(
      tag_detector_.detectTags(cv_image_,camera_info));

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    tag_detector_.drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

} // namespace apriltags2_ros
