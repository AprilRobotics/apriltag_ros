#ifndef APRILTAGS2_ROS_TAG_DETECTOR_H
#define APRILTAGS2_ROS_TAG_DETECTOR_H

#include <string>
#include <sstream>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>

#include <apriltags2_msgs/AprilTagDetectionPose.h>
#include <apriltags2_msgs/AprilTagDetectionPoseArray.h>
#include <apriltags2_msgs/AprilTagDetectionArray.h>
#include <apriltags2_msgs/AprilTagDetection.h>

#include "apriltag.h"
#include "standalone_tag_description.h"
#include "tag_bundle_member.h"
#include "tag_bundle_description.h"
#include "tag_settings.h"

#include "common/homography.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "tag16h5.h"

using namespace apriltags2_msgs;
using namespace sensor_msgs;
using namespace cv_bridge;
using namespace std;

namespace apriltags2_ros {

    class TagDetector {
    protected:
        // Camera Information
        CvImageConstPtr cv_image;
        CvImagePtr cv_image_drawn;
        CameraInfoConstPtr camera_info;
        AprilTagDetectionArray detectionArray;
        AprilTagDetectionArray* detectionArrayPtr;

        // AprilTags 2 code's attributes
        tag_settings settings;

        // TF Publishing
        bool publish_tf;
        tf::TransformBroadcaster tf_pub;
        string camera_tf_frame;

        // Detections sorting
        static int idComparison(const void *first, const void *second);

        // Removes detections of tags with the same ID
        void removeDuplicates(zarray_t *);

    public:
        TagDetector(ros::NodeHandle nh);
        ~TagDetector();

        // AprilTags 2 objects
        apriltag_family_t *family;
        apriltag_detector_t *detector;

        geometry_msgs::PoseWithCovarianceStamped makeTagPose(
                const Eigen::Matrix4d &transform,
                const Eigen::Quaternion<double> rot_quaternion,
                const std_msgs::Header &header);

        // Detects the tags
        void detectTags(AprilTagDetectionArray& detectionArray_msg, const sensor_msgs::ImageConstPtr& image_rect, const sensor_msgs::CameraInfoConstPtr& camera_info);

        // Detect tags in an image
        void findTagPose(AprilTagDetectionPoseArray& detectionPoseArray, const AprilTagDetectionArray detectionArray);

        //

        // Get the pose of the tag in the camera frame
        // Returns homogeneous transformation matrix [R,t;[0 0 0 1]] which
        // takes a point expressed in the tag frame to the same point
        // expressed in the camera frame. As usual, R is the (passive)
        // rotation from the tag frame to the camera frame and t is the
        // vector from the camera frame origin to the tag frame origin,
        // expressed in the camera frame.
        Eigen::Matrix4d getRelativeTransform(std::vector<cv::Point3d> objectPoints,
                                             std::vector<cv::Point2d> imagePoints, double fx, double fy,
                                             double cx, double cy) const;

        void addImagePoints(matd_t& H, std::vector<cv::Point2d> &imagePoints) const;

        void addObjectPoints(double s, cv::Matx44d T_oi, std::vector<cv::Point3d> &objectPoints) const;

        // Draw the detected tags' outlines and payload values on the image
        void drawDetections(const AprilTagDetectionArray& detectionArray, CvImagePtr& img);
    };

}

#endif
