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
/**
 ** single_image_detector.hpp **************************************************
 *
 * Wrapper class of TagDetector class which calls TagDetector::detectTags on a
 * an image stored at a specified load path and stores the output at a specified
 * save path.
 *
 * $Revision: 1.0 $
 * $Date: 2017/12/17 13:33:40 $
 * $Author: dmalyuta $
 *
 * Originator:        Danylo Malyuta, JPL
 ******************************************************************************/

#ifndef APRILTAG_ROS__SINGLE_IMAGE_DETECTOR_HPP_
#define APRILTAG_ROS__SINGLE_IMAGE_DETECTOR_HPP_

#include <memory>

#include "apriltag_ros/common_functions.hpp"
#include "apriltag_msgs/srv/analyze_single_image.hpp"

namespace apriltag_ros
{

class SingleImageDetector : public rclcpp::Node
{
private:
  std::unique_ptr<TagDetector> tag_detector_;
  rclcpp::Service<apriltag_msgs::srv::AnalyzeSingleImage>::SharedPtr
    single_image_analysis_service_;

  rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr
    tag_detections_publisher_;

public:
  explicit SingleImageDetector(const rclcpp::NodeOptions & node_options);

  // The function which provides the single image analysis service
  bool analyzeImage(
    const std::shared_ptr<rmw_request_id_t> request_header,
    apriltag_msgs::srv::AnalyzeSingleImage::Request::SharedPtr request,
    apriltag_msgs::srv::AnalyzeSingleImage::Response::SharedPtr response);
};

}  // namespace apriltag_ros

#endif  // APRILTAG_ROS__SINGLE_IMAGE_DETECTOR_HPP_
