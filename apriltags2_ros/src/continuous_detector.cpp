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

#include <apriltags2_ros/continuous_detector.h>

namespace apriltags2_ros {

	ContinuousDetector::ContinuousDetector(ros::NodeHandle& nh) :
		TagDetector(nh), it(nh) {

		nh.param<bool>("publish_tag_detections_image", draw_tag_detections_image, false);

		// Subscribers
		camera_image_subscriber = it.subscribeCamera("image_rect", 2, &ContinuousDetector::imageCallback, this);

		// Publishers
		tag_detections_poses_publisher = nh.advertise<AprilTagDetectionArray>("tag_detections", 5);
	}

	void ContinuousDetector::imageCallback( const sensor_msgs::ImageConstPtr& image_rect, const sensor_msgs::CameraInfoConstPtr& camera_info) {
		// Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
		// AprilTags 2 on the iamge
		try {
			this->cv_image = cv_bridge::toCvShare(image_rect, sensor_msgs::image_encodings::BGR8);
			this->camera_info = camera_info;

            AprilTagDetectionArray detectionArray;
            detectTags(detectionArray, image_rect, camera_info);
            tag_detections_poses_publisher.publish(detectionArray);

		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

} // namespace apriltags2_ros
