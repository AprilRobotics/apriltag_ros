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

#include "apriltag_ros/common_functions.hpp"
#include "apriltag_ros_interfaces/srv/analyze_single_image.hpp"

bool getRosParameter (rclcpp::Node::SharedPtr nh, std::string name, double& param)
{
  // Write parameter "name" from ROS Parameter Server into param
  // Return true if successful, false otherwise
  if (pnh.hasParam(name.c_str()))
  {
    pnh.getParam(name.c_str(), param);
    ROS_INFO_STREAM("Set camera " << name.c_str() << " = " << param);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Could not find " << name.c_str() << " parameter!");
    return false;
  }
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("single_image_client_node");

  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
    node->create_client<apriltag_ros_interfaces::srv::AnalyzeSingleImage>("single_image_tag_detection");

  auto request = std::make_shared<apriltag_ros_interfaces::srv::AnalyzeSingleImage::Request>();

  // Write desired path where to get the image
  request->full_path_where_to_get_image = "/full/path/of/image";

  // Write desired path where to save the image
  request->full_path_where_to_save_image = "/full/path/of/image";

  // Write camera parameters
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  request->camera_info.K[0] = fx;
  request->camera_info.K[2] = cx;
  request->camera_info.K[4] = fy;
  request->camera_info.K[5] = cy;
  request->camera_info.K[8] = 1.0;
  request->camera_info.P[0] = fx;
  request->camera_info.P[2] = cx;
  request->camera_info.P[5] = fy;
  request->camera_info.P[6] = cy;
  request->camera_info.P[10] = 1.0;

  // Call the service (detect tags in the image specified by the
  // image_load_path)
  if (client.call(service))
  {
    // use parameter run_quielty=false in order to have the service
    // print out the tag position and orientation
    if (service.response.tag_detections.detections.size() == 0)
    {
      ROS_WARN_STREAM("No detected tags!");
    }
  }
  else
  {
    ROS_ERROR("Failed to call service single_image_tag_detection");
    return 1;
  }

	rclcpp::shutdown();
  return 0; // happy ending
}
