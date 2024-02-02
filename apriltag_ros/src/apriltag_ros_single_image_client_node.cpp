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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("single_image_client_node");

    auto client = node->create_client<apriltag_ros_interfaces::srv::AnalyzeSingleImage>("single_image_tag_detection");

    auto request = std::make_shared<apriltag_ros_interfaces::srv::AnalyzeSingleImage::Request>();
    request->full_path_where_to_get_image = argv[1];
    request->full_path_where_to_save_image = argv[2];

    // Replicate sensor_msgs/CameraInfo message
    request->camera_info.distortion_model = "plumb_bob";

    // Declare parameters with default values
    node->declare_parameter<double>("fx", 652.7934615847107);
    node->declare_parameter<double>("fy", 653.9480389077635);
    node->declare_parameter<double>("cx", 307.1288710375904);
    node->declare_parameter<double>("cy", 258.7823279214385);

    // Get parameters
    double fx, fy, cx, cy;
    node->get_parameter("fx", fx);
    node->get_parameter("fy", fy);
    node->get_parameter("cx", cx);
    node->get_parameter("cy", cy);

    // Intrinsic camera matrix for the raw (distorted) images
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

    // Call the service (detect tags in the image specified by the
    // image_load_path)
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Service call failed.");
        return 1;
    }

    auto result = result_future.get();
    auto tag_detections = result->tag_detections;
    if (tag_detections.detections.size() == 0)
    {
        RCLCPP_WARN(node->get_logger(), "No detected tags!");
    }

    rclcpp::shutdown();
    return 0; // happy ending
}
