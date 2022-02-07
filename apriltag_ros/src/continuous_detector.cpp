// Copyright (c) 2017, California Institute of Technology.
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// The views and conclusions contained in the software and documentation are
// those of the authors and should not be interpreted as representing official
// policies, either expressed or implied, of the California Institute of
// Technology.

#include "apriltag_ros/continuous_detector.hpp"

#include <memory>

namespace apriltag_ros
{
ContinuousDetector::ContinuousDetector(const rclcpp::NodeOptions & node_options)
: Node("continuous_detector", node_options)
{
  tag_detector_ = std::make_unique<TagDetector>(this);
  draw_tag_detections_image_ = declare_parameter(
    "publish_tag_detections_image", false);

  const auto transport_hint = declare_parameter("transport_hint", "raw");

  camera_image_subscriber_ = image_transport::create_camera_subscription(
    this, "~/image_rect",
    std::bind(
      &ContinuousDetector::imageCallback,
      this, std::placeholders::_1, std::placeholders::_2), transport_hint);
  tag_detections_publisher_ = create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>(
    "~/tag_detections", 1);
  if (draw_tag_detections_image_) {
    tag_detections_image_publisher_ = image_transport::create_publisher(
      this, "~/tag_detections_image");
  }
}

void ContinuousDetector::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_rect,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info)
{
  // Lazy updates:
  // When there are no subscribers _and_ when tf is not published,
  // skip detection.
  if (tag_detections_publisher_->get_subscription_count() == 0 &&
    tag_detections_publisher_->get_intra_process_subscription_count() == 0 &&
    tag_detections_image_publisher_.getNumSubscribers() == 0 &&
    !tag_detector_->get_publish_tf())
  {
    // ROS_INFO_STREAM("No subscribers and no tf publishing, skip processing.");
    return;
  }

  // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
  // AprilTag 2 on the iamge
  try {
    cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Publish detected tags in the image by AprilTag 2
  tag_detections_publisher_->publish(
    tag_detector_->detectTags(cv_image_, camera_info));

  // Publish the camera image overlaid by outlines of the detected tags and
  // their payload values
  if (draw_tag_detections_image_) {
    tag_detector_->drawDetections(cv_image_);
    tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
  }
}

}  // namespace apriltag_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_ros::ContinuousDetector)
