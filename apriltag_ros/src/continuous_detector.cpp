/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/continuous_detector.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(apriltag_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltag_ros
{
void ContinuousDetector::onInit ()
{
  ros::NodeHandle& nh = getMTNodeHandle();
  ros::NodeHandle& pnh = getMTPrivateNodeHandle();

  //Parameters
  pnh.param("output_frequency", output_frequency_, 10.0);
  pnh.param("is_rectified", is_rectified_, true);
  
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, 
      "publish_tag_detections_image", false);

  //Get camera information
  XmlRpc::XmlRpcValue cameraImage;
    pnh.param("camera_image", cameraImage, cameraImage);
  XmlRpc::XmlRpcValue cameraInfo;
    pnh.param("camera_info", cameraInfo, cameraInfo);

  for(int i =0; (i < cameraImage.size()) && (i < cameraInfo.size()); i++)
  {
    camera_.emplace_back(nh, pnh, cameraImage[i], cameraInfo[i]);
  }

  //Camera subscribers
  for(int i=0; i < camera_.size(); i++)
  {
    //Camera
    camera_[i].cameraSync->registerCallback(boost::bind(&ContinuousDetector::imageCallback, this, _1, _2, i));
  }

  //Publisher
  it_ = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nh));
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  pub_timer_ = nh.createTimer(ros::Duration(1.0/output_frequency_), boost::bind(&ContinuousDetector::publishDetections, this, _1));
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }

}

void ContinuousDetector::imageCallback (
    const sensor_msgs::ImageConstPtr& camera_image,
    const sensor_msgs::CameraInfoConstPtr& camera_info, 
    int index)
{
  
  //ROS_INFO("image callback %d", index);
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_.getNumSubscribers() == 0 &&
      tag_detections_image_publisher_.getNumSubscribers() == 0 &&
      !camera_[index].tag_detector->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  //extract camera instrinsics from camera_info
  cv::Mat camera_instrinsics = cv::Mat(3, 3, CV_64F);
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      camera_instrinsics.at<double>(row, col) = camera_info->K[row * 3 + col];
    }
  }

  cv::Mat distortion_coefficients = cv::Mat(1, 5, CV_64F);
  for (int col = 0; col < 5; col++)
  {
    distortion_coefficients.at<double>(col) = camera_info->D[col];
  }

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag on the image
  try
  {
    camera_[index].cv_image = cv_bridge::toCvCopy(camera_image, camera_image->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //Perform detections
  AprilTagDetectionArray detections = camera_[index].tag_detector->detectTags(camera_[index].cv_image,camera_info);
  
  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_)
  {
    camera_[index].tag_detector->drawDetections(camera_[index].cv_image);
    tag_detections_image_publisher_.publish(camera_[index].cv_image->toImageMsg());
  }

  // Merge detections into common array
  std::lock_guard<std::recursive_mutex> myLock(mutex_);
  if(tag_detection_array_.detections.size() <= 1) tag_detection_array_.header = detections.header;
  for(int i =0 ; i < detections.detections.size(); i++)
  {
    tag_detection_array_.detections.push_back(detections.detections[i]);
  }
}

void ContinuousDetector::publishDetections(const ros::TimerEvent& event)
{
  std::lock_guard<std::recursive_mutex> myLock(mutex_);

  tag_detections_publisher_.publish(tag_detection_array_);

  tag_detection_array_.detections.clear();
}

} // namespace apriltag_ros
