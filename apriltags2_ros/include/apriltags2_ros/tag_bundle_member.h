#ifndef APRILTAGS2_ROS_TAG_BUNDLE_MEMBER_H
#define APRILTAGS2_ROS_TAG_BUNDLE_MEMBER_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

// Stores the properties of a tag member of a bundle
struct TagBundleMember {
	int id; // Payload ID
	double size; // [m] Side length
	cv::Matx44d T_oi; // Rigid transform from tag i frame to bundle origin frame
};

#endif
