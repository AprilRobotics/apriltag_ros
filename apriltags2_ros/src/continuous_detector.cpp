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

#include "apriltags2_ros/continuous_detector.h"

//#include <pluginlib/class_list_macros.h>

//PLUGINLIB_EXPORT_CLASS(apriltags2_ros::ContinuousDetector, nodelet::Nodelet);

namespace apriltags2_ros
{

ContinuousDetector::ContinuousDetector()
{
}

ContinuousDetector::ContinuousDetector(ros::NodeHandle &nh_, ros::NodeHandle &pnh_)
{
  //Get Node Handles
  ros::NodeHandle &nh = nh_;
  ros::NodeHandle &pnh = pnh_;

  //Param
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, "publish_tag_detections_image", false);

  //Create Tag detector
  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));

  //Subscriber and Publishers
  it_ = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh));
  camera_image_subscriber_ = it_->subscribeCamera("image", 1, &ContinuousDetector::imageCallback, this);
  tag_detections_publisher_ = nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }

  //Service
  aprilDetectorOn_ = true;
  tfStopDetectorSrv_ = nh.advertiseService("/aprilTag_stopDetector", &ContinuousDetector::tfStopDetectorCallback, this);
  tfRestartDetectorSrv_ = nh.advertiseService("/aprilTag_startDetector", &ContinuousDetector::tfRestartDetectorCallback, this);
}

/*void ContinuousDetector::onInit()
{
  //Get Node Handles
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();

  //Param
  draw_tag_detections_image_ = getAprilTagOption<bool>(pnh, "publish_tag_detections_image", false);

  //Create Tag detector
  tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(pnh));

  //Subscriber and Publishers
  it_ = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh));
  camera_image_subscriber_ = it_->subscribeCamera("image", 1, &ContinuousDetector::imageCallback, this);
  tag_detections_publisher_ = nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  if (draw_tag_detections_image_)
  {
    tag_detections_image_publisher_ = it_->advertise("tag_detections_image", 1);
  }

  //Service
  aprilDetectorOn_ = true;
  tfStopDetectorSrv_ = nh.advertiseService("/aprilTag_stopDetector", &ContinuousDetector::tfStopDetectorCallback, this);
  tfRestartDetectorSrv_ = nh.advertiseService("/aprilTag_startDetector", &ContinuousDetector::tfRestartDetectorCallback, this);
}*/

void ContinuousDetector::imageCallback(const sensor_msgs::ImageConstPtr &image, const sensor_msgs::CameraInfoConstPtr &camera_info)
{

  if (aprilDetectorOn_)
  {
    // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
    // AprilTags 2 on the iamge
    try
    {
      cv_image_ = cv_bridge::toCvCopy(image, image->encoding);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Publish detected tags in the image by AprilTags 2
    tag_detections_publisher_.publish(tag_detector_->detectTags(cv_image_, camera_info));

    // Publish the camera image overlaid by outlines of the detected tags and their payload values
    if (draw_tag_detections_image_ && tag_detections_image_publisher_.getNumSubscribers() > 0) //CUSTOMIZATION
    {
      tag_detector_->drawDetections(cv_image_);
      tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
    }
  }
  else
    tag_detector_->tfRepublish(image->header.stamp);
}

//CUSTOMIZATION
bool ContinuousDetector::tfStopDetectorCallback(std_srvs::Empty::Request & /*request*/, std_srvs::Empty::Response & /*response*/)
{
  ROS_WARN_ONCE("AprilTag2 Detection STOPPED, only stored tf will be published from now onwards");
  aprilDetectorOn_ = false;
}

bool ContinuousDetector::tfRestartDetectorCallback(std_srvs::Empty::Request & /*request*/, std_srvs::Empty::Response & /*response*/)
{
  ROS_WARN_ONCE("AprilTag2 Detection RESTARTED, live tf published");
  aprilDetectorOn_ = true;
}
//CUSTOMIZATION

} // namespace apriltags2_ros
