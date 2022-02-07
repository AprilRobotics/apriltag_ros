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

#include <memory>
#include <string>

#include "apriltag_ros/common_functions.hpp"
#include "apriltag_msgs/srv/analyze_single_image.hpp"

namespace apriltag_ros
{
class SingleImageClient : public rclcpp::Node
{
public:
  explicit SingleImageClient(const rclcpp::NodeOptions & node_options);
};

SingleImageClient::SingleImageClient(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node("single_image_client", node_options)
{
  using std::chrono_literals::operator""s;

  auto client = create_client<apriltag_msgs::srv::AnalyzeSingleImage>(
    "single_image_tag_detection");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for service...");
  }

  // Get the request parameters
  auto request = std::make_shared<apriltag_msgs::srv::AnalyzeSingleImage::Request>();
  request->full_path_where_to_get_image = declare_parameter<std::string>("image_load_path");
  request->full_path_where_to_save_image = declare_parameter<std::string>("image_save_path");
  // Replicate sensors_msgs/CameraInfo message (must be up-to-date with the
  // analyzed image!)
  request->camera_info.distortion_model = "plumb_bob";
  // Intrinsic camera matrix for the raw (distorted) images
  const auto fx = declare_parameter<double>("fx");
  const auto cx = declare_parameter<double>("cx");
  const auto fy = declare_parameter<double>("fy");
  const auto cy = declare_parameter<double>("cy");
  request->camera_info.k[0] = fx;
  request->camera_info.k[2] = cx;
  request->camera_info.k[4] = fy;
  request->camera_info.k[5] = cy;
  request->camera_info.k[8] = 1.0;
  request->camera_info.p[0] = fx;
  request->camera_info.p[2] = cx;
  request->camera_info.p[5] = fy;
  request->camera_info.p[6] = cy;
  request->camera_info.p[10] = 1.0;

  auto response = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(
      get_node_base_interface(), response) == rclcpp::FutureReturnCode::SUCCESS)
  {
    // use parameter run_quielty=false in order to have the service
    // print out the tag position and orientation
    if (response.get()->tag_detections.detections.size() == 0) {
      RCLCPP_WARN_STREAM(get_logger(), "No detected tags!");
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to call service single_image_tag_detection");
  }

  rclcpp::shutdown();
}
}  // namespace apriltag_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_ros::SingleImageClient)
