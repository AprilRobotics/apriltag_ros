#ifndef APRILTAGS2_ROS_TAG_DETECTOR_H
#define APRILTAGS2_ROS_TAG_DETECTOR_H

#include <string>
#include <sstream>
#include <vector>
#include <map>

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
#include <apriltags2_ros/ZArray.h>
#include "apriltag.h"
#include "standalone_tag_description.h"
#include "tag_bundle_member.h"

#include "common/homography.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "tag16h5.h"

namespace apriltags2_ros {

    class TagBundleDescription;

    class TagDetector {
    private:
        // Detections sorting
        static int idComparison(const void *first, const void *second);

        // Remove detections of tags with the same ID
        void removeDuplicates();

        // Camera Information
        cv_bridge::CvImagePtr image_;
        sensor_msgs::CameraInfoConstPtr camera_info_;

        // AprilTags 2 code's attributes
        std::string family_;
        int border_;
        int threads_;
        double decimate_;
        double blur_;
        int refine_edges_;
        int refine_decode_;
        int refine_pose_;
        int debug_;

        // Other members
        std::map<int, StandaloneTagDescription> standalone_tag_descriptions_;
        std::vector<TagBundleDescription> tag_bundle_descriptions_;
        bool run_quietly_;
        bool publish_tf_;
        tf::TransformBroadcaster tf_pub_;
        std::string camera_tf_frame_;

    public:

        TagDetector(ros::NodeHandle pnh);

        ~TagDetector();

        // AprilTags 2 objects
        apriltag_family_t *tf_;
        apriltag_detector_t *td_;
        zarray_t *detections_;

        // Store standalone and bundle tag descriptions
        std::map<int, StandaloneTagDescription> parseStandaloneTags(
                XmlRpc::XmlRpcValue &standalone_tag_descriptions);

        std::vector<TagBundleDescription> parseTagBundles(
                XmlRpc::XmlRpcValue &tag_bundles);

        double xmlRpcGetDouble(XmlRpc::XmlRpcValue &xmlValue,
                               std::string field) const;

        double xmlRpcGetDoubleWithDefault(XmlRpc::XmlRpcValue &xmlValue,
                                          std::string field, double defaultValue) const;

        bool findStandaloneTagDescription(int id,
                                          StandaloneTagDescription *&descriptionContainer, bool printWarning =
        true);

        geometry_msgs::PoseWithCovarianceStamped makeTagPose(
                const Eigen::Matrix4d &transform,
                const Eigen::Quaternion<double> rot_quaternion,
                const std_msgs::Header &header);

        // Update the camera info simply
        void updateCameraInfo(const cv_bridge::CvImagePtr &image,
                              const sensor_msgs::CameraInfoConstPtr &camera_info);

        // Detect tags in an image
        AprilTagDetectionArray detectTags(const apriltags2_ros::ZArrayConstPtr zarray);

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

        void addImagePoints(apriltag_detection_t *detection,
                            std::vector<cv::Point2d> &imagePoints) const;

        void addObjectPoints(double s, cv::Matx44d T_oi,
                             std::vector<cv::Point3d> &objectPoints) const;

        // Draw the detected tags' outlines and payload values on the image
        void drawDetections(cv_bridge::CvImagePtr image);
    };

    template<typename T>
    T getAprilTagOption(ros::NodeHandle &pnh, const std::string &param_name,
                        const T &default_val) {
        T param_val;
        pnh.param<T>(param_name, param_val, default_val);
        return param_val;
    }

}

#endif
