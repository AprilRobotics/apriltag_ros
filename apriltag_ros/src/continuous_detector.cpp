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

#include "apriltag_ros/continuous_detector.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


using namespace apriltag_ros;

ContinuousDetector::ContinuousDetector(const rclcpp::NodeOptions & options)
: nh_(std::make_shared<rclcpp::Node>("apriltag_node", options)), custom_qos_(1)
{
    rclcpp::uninstall_signal_handlers();

    // Declare and get parameters
    draw_tag_detections_image_ = nh_->declare_parameter<bool>("publish_tag_detections_image", false);
    std::string tag_detections_image_topic = nh_->declare_parameter<std::string>("tag_detections_image_topic", "tag_detections_image");
    std::string image_topic = nh_->declare_parameter<std::string>("image_topic", "color/image_raw");
    int queue_size = nh_->declare_parameter<int>("queue_size", 1);
    std::string tag_detections_topic = nh_->declare_parameter<std::string>("tag_detections_topic", "tag_detections");
    
    tag_detector_ = std::shared_ptr<TagDetector>(new TagDetector(nh_));


    // Image_transport
    it_ = std::shared_ptr<image_transport::ImageTransport>(
        new image_transport::ImageTransport(nh_));

    camera_image_subscriber_ = it_->subscribeCamera(image_topic, queue_size,
                            &ContinuousDetector::ImageCallback, this,
                            new image_transport::TransportHints(nh_.get(), "raw", "transport_hint"));

    tag_detections_publisher_ = nh_->create_publisher<apriltag_ros_interfaces::msg::AprilTagDetectionArray>(tag_detections_topic, 10);

    if (draw_tag_detections_image_)
    {
        tag_detections_image_publisher_ = it_->advertise(tag_detections_image_topic, 1);
    }

}


void ContinuousDetector::ImageCallback (
    const sensor_msgs::msg::Image::ConstSharedPtr& image_rect,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info)
{
    // Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
    // AprilTag 2 on the iamge
    try
    {
        cv_image_ = cv_bridge::toCvCopy(image_rect, image_rect->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(nh_->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Publish detected tags in the image by AprilTag 2
    tag_detections_publisher_->publish(
        tag_detector_->detectTags(cv_image_,camera_info));

    // Publish the camera image overlaid by outlines of the detected tags and
    // their payload values
    if (draw_tag_detections_image_)
    {
        tag_detector_->drawDetections(cv_image_);
        tag_detections_image_publisher_.publish(cv_image_->toImageMsg());
    }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
ContinuousDetector::get_node_base_interface() const
{
    return this->nh_->get_node_base_interface();
}


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(apriltag_ros::ContinuousDetector)