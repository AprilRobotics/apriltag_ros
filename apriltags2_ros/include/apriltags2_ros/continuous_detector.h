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
 *
 ** continuous_detector.h ******************************************************
 *
 * Wrapper class of TagDetector class which calls TagDetector::detectTags on
 * each newly arrived image published by a camera.
 *
 * $Revision: 1.0 $
 * $Date: 2017/12/17 13:25:52 $
 * $Author: dmalyuta $
 *
 * Originator:        Danylo Malyuta, JPL
 ******************************************************************************/

#ifndef APRILTAGS2_ROS_CONTINUOUS_DETECTOR_H
#define APRILTAGS2_ROS_CONTINUOUS_DETECTOR_H

#include "apriltags2_ros/AprilTagDetection.h"
#include "apriltags2_ros/AprilTagDetectionArray.h"
#include "apriltag.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "standalone_tag_description.h"
#include "tag_bundle_member.h"
#include "tag_detector.h"

#include <apriltags2_ros/tag_bundle_description.h>
#include <apriltags2_ros/ZArray.h>

namespace apriltags2_ros {

class ContinuousDetector {
public:
	ContinuousDetector(ros::NodeHandle& nh);

	void imageCallback(const sensor_msgs::ImageConstPtr& image_rect, const sensor_msgs::CameraInfoConstPtr& camera_info);

	void zArrayCallback(const apriltags2_ros::ZArrayConstPtr zarray);

private:
	TagDetector tag_detector_;
	bool draw_tag_detections_image_;
	cv_bridge::CvImagePtr cv_image_;

	image_transport::ImageTransport it_;
	image_transport::CameraSubscriber camera_image_subscriber_;
	image_transport::Publisher tag_detections_image_publisher_;
	ros::Publisher tag_detections_publisher_;
	ros::Subscriber z_array_subscriber_;
};

} // namespace apriltags2_ros

#endif // APRILTAGS2_ROS_CONTINUOUS_DETECTOR_H
