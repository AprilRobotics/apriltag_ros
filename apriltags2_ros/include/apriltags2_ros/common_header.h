#ifndef APRILTAGS2_WRAPPER_COMMON_HEADER_H
#define APRILTAGS2_WRAPPER_COMMON_HEADER_H

#include <string>
#include <sstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <XmlRpcException.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <apriltags2_ros/AprilTagDetection.h>
#include <apriltags2_ros/AprilTagDetectionArray.h>

#include <common/homography.h>
#include <apriltag.h>
#include <tag36h11.h>
#include <tag36h10.h>
#include <tag36artoolkit.h>
#include <tag25h9.h>
#include <tag25h7.h>

namespace apriltags2_ros {

template<typename T>
T apriltag_getopt(ros::NodeHandle& pnh, const std::string& param_name, const T & default_val) {
  T param_val;
  pnh.param<T>(param_name, param_val, default_val);
  return param_val;
}

class TagDescription {
 public:

  TagDescription(int id, double size, std::string &frame_name) : id_(id), size_(size), frame_name_(frame_name) {}

  double size() { return size_; }
  int id() { return id_; }
  std::string& frame_name() { return frame_name_; }

 private:
  // Tag description
  int id_;
  double size_;
  std::string frame_name_;
};

class TagDetector {
 public:

  TagDetector(ros::NodeHandle pnh);
  ~TagDetector();

  // Create a map of tag payload value to parameters of the tag (e.g. its side length)
  std::map<int, TagDescription> parse_tag_descriptions(XmlRpc::XmlRpcValue& april_tag_descriptions);

  // Detect tags in an image
AprilTagDetectionArray detect_tags(const cv_bridge::CvImagePtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info);

  // Get the pose of the tag in the camera frame
  // 
  // Returns homogeneous transformation matrix [R,t;[0 0 0 1]] which
  // takes a point expressed in the tag frame to the same point
  // expressed in the camera frame. As usual, R is the (passive)
  // rotation from the tag frame to the camera frame and t is the
  // vector from the camera frame origin to the tag frame origin,
  // expressed in the camera frame.
  Eigen::Matrix4d getRelativeTransform(apriltag_detection_t *detection, double tag_size, double fx, double fy, double cx, double cy) const;

  // Draw the detected tags' outlines and payload values on the image
  void draw_detections(cv_bridge::CvImagePtr image);

 private:

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

  // AprilTags 2 objects
  apriltag_family_t *tf_;
  apriltag_detector_t *td_;
  zarray_t *detections_;

  // Other members
  std::string sensor_frame_id_;
  bool projected_optics_;
  std::map<int, TagDescription> descriptions_;
  bool run_quietly_;
  bool publish_tf_;
  tf::TransformBroadcaster tf_pub_;
  Eigen::Quaternion<double> rot_quaternion_previous_;

};

}

#endif //APRILTAGS2_WRAPPER_COMMON_HEADER_H
