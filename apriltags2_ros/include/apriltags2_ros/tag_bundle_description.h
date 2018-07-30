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
 ** common_functions.h *********************************************************
 *
 * Wrapper classes for AprilTag standalone and bundle detection. Main function
 * is TagDetector::detectTags which wraps the call to core AprilTags 2
 * algorithm, apriltag_detector_detect().
 *
 * $Revision: 1.0 $
 * $Date: 2017/12/17 13:23:14 $
 * $Author: dmalyuta $
 *
 * Originator:        Danylo Malyuta, JPL
 ******************************************************************************/

#ifndef APRILTAGS2_ROS_TAG_BUNDLE_DESCRIPTION_H
#define APRILTAGS2_ROS_TAG_BUNDLE_DESCRIPTION_H

#include <string>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <XmlRpcException.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>

#include "apriltags2_ros/AprilTagDetection.h"
#include "apriltags2_ros/AprilTagDetectionArray.h"
#include "apriltag.h"

#include "standalone_tag_description.h"
#include "tag_bundle_member.h"
#include "tag_detector.h"

using namespace std;

namespace apriltags2_ros {

class TagBundleDescription {
public:
	std::map<int, int> id2idx_; // (id2idx_[<tag ID>]=<index in tags_>) mapping

	TagBundleDescription(std::string name) :
			name_(name) {
	}

	void addMemberTag(int id, double size, cv::Matx44d T_oi) {
		TagBundleMember member;
		member.id = id;
		member.size = size;
		member.T_oi = T_oi;
		tags_.push_back(member);
		id2idx_[id] = tags_.size() - 1;
	}

	std::string name() const {
		return name_;
	}
	// Get IDs of bundle member tags
	std::vector<int> bundleIds() {
		std::vector<int> ids;
		for (unsigned int i = 0; i < tags_.size(); i++) {
			ids.push_back(tags_[i].id);
		}
		return ids;
	}
	// Get sizes of bundle member tags
	std::vector<double> bundleSizes() {
		std::vector<double> sizes;
		for (unsigned int i = 0; i < tags_.size(); i++) {
			sizes.push_back(tags_[i].size);
		}
		return sizes;
	}
	int memberID(int tagID) {
		return tags_[id2idx_[tagID]].id;
	}
	double memberSize(int tagID) {
		return tags_[id2idx_[tagID]].size;
	}
	cv::Matx44d memberT_oi(int tagID) {
		return tags_[id2idx_[tagID]].T_oi;
	}

private:
	// Bundle description
	std::string name_;
	std::vector<TagBundleMember> tags_;
};

} // namespace apriltags2_ros

#endif // APRILTAGS2_ROS_TAG_BUNDLE_DESCRIPTION_H
