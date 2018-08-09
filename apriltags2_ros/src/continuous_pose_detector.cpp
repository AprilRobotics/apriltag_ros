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

#include <apriltags2_ros/continuous_pose_detector.h>
#include <climits>
#include <opencv_apps/Flow.h>
#include <opencv_apps/LKFlowInitializePoints.h>
#include <opencv_apps/Point2D.h>
#include <ctime>
#include <std_srvs/Empty.h>
#include <cassert>

using namespace std;
using namespace opencv_apps;

namespace apriltags2_ros {

ContinuousPoseDetector::ContinuousPoseDetector(ros::NodeHandle& nh) :
		TagDetector(nh), it(nh) {

	nh.param<bool>("publish_tag_detections_image", draw_tag_detections_image, false);
	nh.param<bool>("optical_flow_accelerated", optical_flow_accelerated, false);

	// Subscribers
	camera_image_subscriber = it.subscribeCamera("image_rect", 1, &ContinuousPoseDetector::imageCallback, this);
	april_tag_detection_array_subscriber = nh.subscribe("tag_detections", 5, &ContinuousPoseDetector::location2DCallback, this);
	tag_detections_publisher = nh.advertise<AprilTagDetectionPoseArray>("tag_pose_detections", 1);

	// Publishers
	if (draw_tag_detections_image)
		tag_detections_image_publisher = it.advertise("tag_detections_image", 1);

	if (optical_flow_accelerated) {
        optical_flow_subscriber = nh.subscribe("/lk_flow/flows", 10, &ContinuousPoseDetector::opticalFlowCallback, this);
        optical_flow_client = nh.serviceClient<opencv_apps::LKFlowInitializePoints>("/lk_flow/initialize_points");
    }
}

void ContinuousPoseDetector::imageCallback( const sensor_msgs::ImageConstPtr& image_rect, const sensor_msgs::CameraInfoConstPtr& camera_info) {
	// Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
	// AprilTags 2 on the iamge
	try {
		this->cv_image = cv_bridge::toCvCopy(image_rect, sensor_msgs::image_encodings::BGR8);
		this->camera_info = camera_info;
		this->imageQueue.push_back(this->cv_image);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

int step = 1;
void ContinuousPoseDetector::location2DCallback(const apriltags2_msgs::AprilTagDetectionArrayConstPtr detectionArray) {

	this->detectionArray = *detectionArray;
	this->detectionArrayPtr = &this->detectionArray;

    // Reiniatialize the optical flow points
    if (optical_flow_accelerated) {
        if (step++ % 1 == 0) {
            opencv_apps::LKFlowInitializePoints pts;
            for(auto &det : detectionArray->detections) {
                for (auto &p : det.p) {
                    opencv_apps::Point2D pt;
                    pt.x = p.x;
                    pt.y = p.y;
                    pts.request.points.points.push_back(pt);
                }
                opencv_apps::Point2D pt;
                pt.x = det.c.x;
                pt.y = det.c.y;
                pts.request.points.points.push_back(pt);
            }
            optical_flow_client.call(pts);
        }
    }

	AprilTagDetectionPoseArray detectionPoseArray;
	findTagPose(detectionPoseArray, *detectionArray);
	tag_detections_publisher.publish(detectionPoseArray);

	// Pop all the current transformations before this time stamp
    while(!flowQueue.empty() && flowQueue.back().header.stamp < detectionArray->header.stamp) {
        flowQueue.pop_back();
    }
    detectionArrayStamp = detectionArray->header.stamp;

	// Publish the camera image overlaid by outlines of the detected tags and their payload values
	if (draw_tag_detections_image && !optical_flow_accelerated) {
		drawDetections(this->detectionArray, cv_image);
		tag_detections_image_publisher.publish(cv_image->toImageMsg());
	}
}

float distanceBetweenTwoPoints(float x1, float y1, float x2, float y2) {
	return (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
}

void ContinuousPoseDetector::opticalFlowCallback(const opencv_apps::FlowArrayStampedConstPtr flowArray) {
	if (flowArray->flow.empty())
        return;

	// Move the flow down the queue
	flowQueue.push_front(*flowArray);

	if (this->detectionArrayPtr == nullptr)
		return;

    clock_t start = clock();

    for(auto &flow : flowQueue) {
        cout << flow.header.seq << " ";
    }
    cout << endl;

    // Update the location of the 4 corners based
	AprilTagDetectionArray updatedArray = detectionArray;

    auto pastFlow = flowQueue.rbegin();
    while(pastFlow != flowQueue.rend() && pastFlow->header.stamp <= detectionArrayStamp)
        pastFlow++;

	while(detectionArrayStamp < flowArray->header.stamp) {
        cout << "Test " << detectionArrayStamp << " " << pastFlow->header.stamp << endl;

        for (auto &detection : updatedArray.detections) {
            // For the center point
            float closestDist = numeric_limits<float>::max();
            int closestFlowIndex = 0;

            for (unsigned i = 0; i < flowArray->flow.size(); ++i) {
                float dist = distanceBetweenTwoPoints(detection.c.x, detection.c.y, pastFlow->flow[i].point.x,
                                                      pastFlow->flow[i].point.y);
                if (dist < closestDist) {
                    closestDist = dist;
                    closestFlowIndex = i;
                }
            }

            detection.c.x = detection.c.x + pastFlow->flow[closestFlowIndex].velocity.x;
            detection.c.y = detection.c.y + pastFlow->flow[closestFlowIndex].velocity.y;

//		cout << detection.c.x << " " << detection.c.y << " " << flowArray->flow[closestFlowIndex].point.x << " " << flowArray->flow[closestFlowIndex].point.y << endl;
//		cout << closestFlowIndex << " " << closestDist << " " << flowArray->flow[closestFlowIndex].velocity.x << " " << flowArray->flow[closestFlowIndex].velocity.y << endl;

            for (auto &point : detection.p) {

                // For the center point
                float closestDist = numeric_limits<float>::max();
                int closestFlowIndex = 0;

                for (unsigned i = 0; i < flowArray->flow.size(); ++i) {
                    float dist = distanceBetweenTwoPoints(point.x, point.y, pastFlow->flow[i].point.x,
                                                          pastFlow->flow[i].point.y);
                    if (dist < closestDist) {
                        closestDist = dist;
                        closestFlowIndex = i;
                    }
                }

                point.x = point.x + pastFlow->flow[closestFlowIndex].velocity.x;
                point.y = point.y + pastFlow->flow[closestFlowIndex].velocity.y;
            }
        }

        detectionArrayStamp = pastFlow->header.stamp;
        pastFlow++;
    }

    // Find the object in the image itself
    this->detectionArray = updatedArray;

    // Update the detector
    AprilTagDetectionPoseArray detectionPoseArray;
    findTagPose(detectionPoseArray, detectionArray);
    tag_detections_publisher.publish(detectionPoseArray);

    // Throw out old images
    while(imageQueue.size() > 1 && imageQueue.front()->header.stamp < flowArray->header.stamp) {
        imageQueue.pop_front();
    }

    // Publish the camera image overlaid by outlines of the detected tags and their payload values
    if (draw_tag_detections_image) {
        drawDetections(updatedArray, imageQueue.front());
        tag_detections_image_publisher.publish(imageQueue.front()->toImageMsg());
    }

    clock_t end = clock();
    cout << "Optical Flow Callback took " << (end - start) / (double) CLOCKS_PER_SEC << endl;
}

} // namespace apriltags2_ros
