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

#include "apriltag_ros/single_image_detector.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/msg/header.hpp>

using namespace apriltag_ros;
using std::placeholders::_1;
using std::placeholders::_2;

SingleImageDetector::SingleImageDetector (const rclcpp::NodeOptions & options)
: nh_(std::make_shared<rclcpp::Node>("apriltag_node", options)), custom_qos_(1)
{
    rclcpp::uninstall_signal_handlers();

    // Advertise the single image analysis service
    single_image_analysis_service_ = nh_->create_service<apriltag_ros_interfaces::srv::AnalyzeSingleImage>("single_image_tag_detection",
        std::bind(&SingleImageDetector::analyzeImage, this, _1, _2));

    tag_detections_publisher_ = nh_->create_publisher<apriltag_ros_interfaces::msg::AprilTagDetectionArray>("tag_detections", 1);
    RCLCPP_INFO(nh_->get_logger(),"Ready to do tag detection on single images");
}

void SingleImageDetector::analyzeImage(
    const std::shared_ptr<apriltag_ros_interfaces::srv::AnalyzeSingleImage::Request> request,
    std::shared_ptr<apriltag_ros_interfaces::srv::AnalyzeSingleImage::Response> response)
{

    RCLCPP_INFO(nh_->get_logger(),"[ Summoned to analyze image ]");
    RCLCPP_INFO(nh_->get_logger(),"Image load path: %s",
            request->full_path_where_to_get_image.c_str());
    RCLCPP_INFO(nh_->get_logger(),"Image save path: %s",
            request->full_path_where_to_save_image.c_str());

    // Read the image
    cv::Mat image = cv::imread(request->full_path_where_to_get_image,
                                cv::IMREAD_COLOR);
    if (image.data == NULL)
    {
        // Cannot read image
        RCLCPP_ERROR(nh_->get_logger(),"Could not read image %s", 
                request->full_path_where_to_get_image.c_str());
        return;
    }

    // Detect tags in the image
    cv_bridge::CvImagePtr loaded_image(new cv_bridge::CvImage(std_msgs::msg::Header(),
                                                                "bgr8", image));
    loaded_image->header.frame_id = "camera";
    response->tag_detections =
        tag_detector_->detectTags(loaded_image,sensor_msgs::msg::CameraInfo::ConstSharedPtr(
            new sensor_msgs::msg::CameraInfo(request->camera_info)));

    // Publish detected tags (AprilTagDetectionArray, basically an array of
    // geometry_msgs/PoseWithCovarianceStamped)
    tag_detections_publisher_->publish(response->tag_detections);

    // Save tag detections image
    tag_detector_->drawDetections(loaded_image);
    cv::imwrite(request->full_path_where_to_save_image, loaded_image->image);

    RCLCPP_INFO(nh_->get_logger(),"Done!");

}


rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
SingleImageDetector::get_node_base_interface() const
{
    return this->nh_->get_node_base_interface();
}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_ros::SingleImageDetector)
