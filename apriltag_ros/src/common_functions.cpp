// Copyright (c) 2017, California Institute of Technology.
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// The views and conclusions contained in the software and documentation are
// those of the authors and should not be interpreted as representing official
// policies, either expressed or implied, of the California Institute of
// Technology.

#include <tagStandard52h13.h>
#include <tagStandard41h12.h>
#include <tag36h11.h>
#include <tag25h9.h>
#include <tag16h5.h>
#include <tagCustom48h12.h>
#include <tagCircle21h7.h>
#include <tagCircle49h12.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "apriltag_ros/common_functions.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "rcpputils/asserts.hpp"
#if USE_TF2_GEOMETRY_MSGS_HPP_HEADER
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif

#include "common/homography.h"

namespace apriltag_ros
{

TagDetector::TagDetector(rclcpp::Node * node)
: family_(node->declare_parameter("tag_family", "tag36h11")),
  threads_(node->declare_parameter("tag_threads", 4)),
  decimate_(node->declare_parameter("tag_decimate", 1.0)),
  blur_(node->declare_parameter("tag_blur", 0.0)),
  refine_edges_(node->declare_parameter("tag_refine_edges", 1)),
  debug_(node->declare_parameter("tag_debug", 0)),
  max_hamming_distance_(node->declare_parameter("max_hamming_dist", 2)),
  clock_(node->get_clock()),
  standalone_tag_descriptions_(parseStandaloneTags(node)),
  tag_bundle_descriptions_(parseTagBundles(node)),
  remove_duplicates_(node->declare_parameter("remove_duplicates", true)),
  publish_tf_(node->declare_parameter("publish_tf", false)),
  tf_pub_(std::make_unique<tf2_ros::TransformBroadcaster>(node))
{
  // Define the tag family whose tags should be searched for in the camera
  // images
  if (family_ == "tagStandard52h13") {
    tf_ = tagStandard52h13_create();
  } else if (family_ == "tagStandard41h12") {
    tf_ = tagStandard41h12_create();
  } else if (family_ == "tag36h11") {
    tf_ = tag36h11_create();
  } else if (family_ == "tag25h9") {
    tf_ = tag25h9_create();
  } else if (family_ == "tag16h5") {
    tf_ = tag16h5_create();
  } else if (family_ == "tagCustom48h12") {
    tf_ = tagCustom48h12_create();
  } else if (family_ == "tagCircle21h7") {
    tf_ = tagCircle21h7_create();
  } else if (family_ == "tagCircle49h12") {
    tf_ = tagCircle49h12_create();
  } else {
    RCLCPP_WARN(logger_, "Invalid tag family specified! Aborting");
    exit(1);
  }

  // Create the AprilTag 2 detector
  td_ = apriltag_detector_create();
  apriltag_detector_add_family_bits(td_, tf_, max_hamming_distance_);
  td_->quad_decimate = static_cast<float>(decimate_);
  td_->quad_sigma = static_cast<float>(blur_);
  td_->nthreads = threads_;
  td_->debug = debug_;
  td_->refine_edges = refine_edges_;

  detections_ = nullptr;
}

// destructor
TagDetector::~TagDetector()
{
  // free memory associated with tag detector
  apriltag_detector_destroy(td_);

  // Free memory associated with the array of tag detections
  if(detections_)
  {
    apriltag_detections_destroy(detections_);
  }

  // free memory associated with tag family
  if (family_ == "tagStandard52h13") {
    tagStandard52h13_destroy(tf_);
  } else if (family_ == "tagStandard41h12") {
    tagStandard41h12_destroy(tf_);
  } else if (family_ == "tag36h11") {
    tag36h11_destroy(tf_);
  } else if (family_ == "tag25h9") {
    tag25h9_destroy(tf_);
  } else if (family_ == "tag16h5") {
    tag16h5_destroy(tf_);
  } else if (family_ == "tagCustom48h12") {
    tagCustom48h12_destroy(tf_);
  } else if (family_ == "tagCircle21h7") {
    tagCircle21h7_destroy(tf_);
  } else if (family_ == "tagCircle49h12") {
    tagCircle49h12_destroy(tf_);
  }
}

apriltag_ros::msg::AprilTagDetectionArray TagDetector::detectTags(
  const cv_bridge::CvImagePtr & image,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info)
{
  // Convert image to AprilTag code's format
  cv::Mat gray_image;
  if (image->image.channels() == 1) {
    gray_image = image->image;
  } else {
    cv::cvtColor(image->image, gray_image, CV_BGR2GRAY);
  }
  image_u8_t apriltag_image{
    gray_image.cols,  // width
    gray_image.rows,  // height
    gray_image.cols,  // stride
    gray_image.data   // buf
  };

  image_geometry::PinholeCameraModel camera_model;
  camera_model.fromCameraInfo(camera_info);

  // Get camera intrinsic properties for rectified image.
  double fx = camera_model.fx();  // focal length in camera x-direction [px]
  double fy = camera_model.fy();  // focal length in camera y-direction [px]
  double cx = camera_model.cx();  // optical center x-coordinate [px]
  double cy = camera_model.cy();  // optical center y-coordinate [px]

  // Run AprilTag 2 algorithm on the image
  if (detections_) {
    apriltag_detections_destroy(detections_);
    detections_ = NULL;
  }
  detections_ = apriltag_detector_detect(td_, &apriltag_image);

  // If remove_dulpicates_ is set to true, then duplicate tags are not allowed.
  // Thus any duplicate tag IDs visible in the scene must include at least 1
  // erroneous detection. Remove any tags with duplicate IDs to ensure removal
  // of these erroneous detections
  if (remove_duplicates_) {
    removeDuplicates();
  }

  // Compute the estimated translation and rotation individually for each
  // detected tag
  apriltag_ros::msg::AprilTagDetectionArray tag_detection_array;
  std::vector<std::string> detection_names;
  tag_detection_array.header = image->header;
  std::map<std::string, std::vector<cv::Point3d>> bundleObjectPoints;
  std::map<std::string, std::vector<cv::Point2d>> bundleImagePoints;
  for (int i = 0; i < zarray_size(detections_); i++) {
    // Get the i-th detected tag
    apriltag_detection_t * detection;
    zarray_get(detections_, i, &detection);

    // Bootstrap this for loop to find this tag's description amongst
    // the tag bundles. If found, add its points to the bundle's set of
    // object-image corresponding points (tag corners) for cv::solvePnP.
    // Don't yet run cv::solvePnP on the bundles, though, since we're still in
    // the process of collecting all the object-image corresponding points
    int tagID = detection->id;
    bool is_part_of_bundle = false;
    for (unsigned int j = 0; j < tag_bundle_descriptions_.size(); j++) {
      // Iterate over the registered bundles
      TagBundleDescription bundle = tag_bundle_descriptions_[j];

      if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end()) {
        // This detected tag belongs to the j-th tag bundle (its ID was found in
        // the bundle description)
        is_part_of_bundle = true;
        std::string bundleName = bundle.name();

        //===== Corner points in the world frame coordinates
        double s = bundle.memberSize(tagID) / 2;
        addObjectPoints(
          s, bundle.memberT_oi(tagID),
          bundleObjectPoints[bundleName]);

        //===== Corner points in the image frame coordinates
        addImagePoints(detection, bundleImagePoints[bundleName]);
      }
    }

    // Find this tag's description amongst the standalone tags
    // Print warning when a tag was found that is neither part of a
    // bundle nor standalone (thus it is a tag in the environment
    // which the user specified no description for, or Apriltags
    // misdetected a tag (bad ID or a false positive)).
    StandaloneTagDescription * standaloneDescription;
    if (!findStandaloneTagDescription(
        tagID, standaloneDescription,
        !is_part_of_bundle))
    {
      continue;
    }

    //=================================================================
    // The remainder of this for loop is concerned with standalone tag
    // poses!
    double tag_size = standaloneDescription->size();

    // Get estimated tag pose in the camera frame.
    //
    // Note on frames:
    // The raw AprilTag 2 uses the following frames:
    //   - camera frame: looking from behind the camera (like a
    //     photographer), x is right, y is up and z is towards you
    //     (i.e. the back of camera)
    //   - tag frame: looking straight at the tag (oriented correctly),
    //     x is right, y is down and z is away from you (into the tag).
    // But we want:
    //   - camera frame: looking from behind the camera (like a
    //     photographer), x is right, y is down and z is straight
    //     ahead
    //   - tag frame: looking straight at the tag (oriented correctly),
    //     x is right, y is up and z is towards you (out of the tag).
    // Using these frames together with cv::solvePnP directly avoids
    // AprilTag 2's frames altogether.
    // TODO(someone) solvePnP[Ransac] better?
    std::vector<cv::Point3d> standaloneTagObjectPoints;
    std::vector<cv::Point2d> standaloneTagImagePoints;
    addObjectPoints(tag_size / 2, cv::Matx44d::eye(), standaloneTagObjectPoints);
    addImagePoints(detection, standaloneTagImagePoints);
    Eigen::Matrix4d transform = getRelativeTransform(
      standaloneTagObjectPoints,
      standaloneTagImagePoints,
      fx, fy, cx, cy);
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rot_quaternion(rot);

    geometry_msgs::msg::PoseWithCovarianceStamped tag_pose =
      makeTagPose(transform, rot_quaternion, image->header);

    // Add the detection to the back of the tag detection array
    apriltag_ros::msg::AprilTagDetection tag_detection;
    tag_detection.pose = tag_pose;
    tag_detection.id.push_back(detection->id);
    tag_detection.size.push_back(tag_size);
    tag_detection_array.detections.push_back(tag_detection);
    detection_names.push_back(standaloneDescription->frame_name());
  }

  //=================================================================
  // Estimate bundle origin pose for each bundle in which at least one
  // member tag was detected

  for (unsigned int j = 0; j < tag_bundle_descriptions_.size(); j++) {
    // Get bundle name
    std::string bundleName = tag_bundle_descriptions_[j].name();

    std::map<std::string,
      std::vector<cv::Point3d>>::iterator it =
      bundleObjectPoints.find(bundleName);
    if (it != bundleObjectPoints.end()) {
      // Some member tags of this bundle were detected, get the bundle's
      // position!
      TagBundleDescription & bundle = tag_bundle_descriptions_[j];

      Eigen::Matrix4d transform =
        getRelativeTransform(
        bundleObjectPoints[bundleName],
        bundleImagePoints[bundleName], fx, fy, cx, cy);
      Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
      Eigen::Quaternion<double> rot_quaternion(rot);

      geometry_msgs::msg::PoseWithCovarianceStamped bundle_pose =
        makeTagPose(transform, rot_quaternion, image->header);

      // Add the detection to the back of the tag detection array
      apriltag_ros::msg::AprilTagDetection tag_detection;
      tag_detection.pose = bundle_pose;
      tag_detection.id = bundle.bundleIds();
      tag_detection.size = bundle.bundleSizes();
      tag_detection_array.detections.push_back(tag_detection);
      detection_names.push_back(bundle.name());
    }
  }

  // If set, publish the transform /tf topic
  if (publish_tf_) {
    for (unsigned int i = 0; i < tag_detection_array.detections.size(); i++) {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.frame_id = image->header.frame_id;
      transform_stamped.child_frame_id = detection_names[i];
      transform_stamped.header.stamp = tag_detection_array.detections[i].pose.header.stamp;

      const auto & pose = tag_detection_array.detections[i].pose.pose.pose;
      transform_stamped.transform.translation.x = pose.position.x;
      transform_stamped.transform.translation.y = pose.position.y;
      transform_stamped.transform.translation.z = pose.position.z;

      tf2::Quaternion tf_quaternion;
      tf2::fromMsg(pose.orientation, tf_quaternion);
      transform_stamped.transform.rotation.x = tf_quaternion.x();
      transform_stamped.transform.rotation.y = tf_quaternion.y();
      transform_stamped.transform.rotation.z = tf_quaternion.z();
      transform_stamped.transform.rotation.w = tf_quaternion.w();

      tf_pub_->sendTransform(transform_stamped);
    }
  }

  return tag_detection_array;
}

int TagDetector::idComparison(const void * first, const void * second)
{
  int id1 = (reinterpret_cast<const apriltag_detection_t *>(first))->id;
  int id2 = (reinterpret_cast<const apriltag_detection_t *>(second))->id;
  return (id1 < id2) ? -1 : ((id1 == id2) ? 0 : 1);
}

void TagDetector::removeDuplicates()
{
  zarray_sort(detections_, &idComparison);
  int count = 0;
  bool duplicate_detected = false;
  while (true) {
    if (count > zarray_size(detections_) - 1) {
      // The entire detection set was parsed
      return;
    }
    apriltag_detection_t * detection;
    zarray_get(detections_, count, &detection);
    int id_current = detection->id;
    // Default id_next value of -1 ensures that if the last detection
    // is a duplicated tag ID, it will get removed
    int id_next = -1;
    if (count < zarray_size(detections_) - 1) {
      zarray_get(detections_, count + 1, &detection);
      id_next = detection->id;
    }
    if (id_current == id_next || (id_current != id_next && duplicate_detected)) {
      duplicate_detected = true;
      // Remove the current tag detection from detections array
      int shuffle = 0;
      zarray_remove_index(detections_, count, shuffle);
      if (id_current != id_next) {
        RCLCPP_WARN_STREAM(
          logger_,
          "Pruning tag ID " << id_current << " because it "
            "appears more than once in the image.");
        duplicate_detected = false;  // Reset
      }
      continue;
    } else {
      count++;
    }
  }
}

void TagDetector::addObjectPoints(
  double s, cv::Matx44d T_oi, std::vector<cv::Point3d> & objectPoints) const
{
  // Add to object point vector the tag corner coordinates in the bundle frame
  // Going counterclockwise starting from the bottom left corner
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, -s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, -s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, s, 0, 1));
}

void TagDetector::addImagePoints(
  apriltag_detection_t * detection,
  std::vector<cv::Point2d> & imagePoints) const
{
  // Add to image point vector the tag corners in the image frame
  // Going counterclockwise starting from the bottom left corner
  double tag_x[4] = {-1, 1, 1, -1};
  double tag_y[4] = {1, 1, -1, -1};  // Negated because AprilTag tag local
                                     // frame has y-axis pointing DOWN
                                     // while we use the tag local frame
                                     // with y-axis pointing UP
  for (int i = 0; i < 4; i++) {
    // Homography projection taking tag local frame coordinates to image pixels
    double im_x, im_y;
    homography_project(detection->H, tag_x[i], tag_y[i], &im_x, &im_y);
    imagePoints.push_back(cv::Point2d(im_x, im_y));
  }
}

Eigen::Matrix4d TagDetector::getRelativeTransform(
  std::vector<cv::Point3d> objectPoints,
  std::vector<cv::Point2d> imagePoints,
  double fx, double fy, double cx, double cy) const
{
  // perform Perspective-n-Point camera pose estimation using the
  // above 3D-2D point correspondences
  cv::Mat rvec, tvec;
  cv::Matx33d cameraMatrix(fx, 0, cx,
    0, fy, cy,
    0, 0, 1);
  cv::Vec4f distCoeffs(0, 0, 0, 0);  // distortion coefficients
  // TODO(someone): Perhaps something like SOLVEPNP_EPNP would be faster? Would
  // need to first check WHAT is a bottleneck in this code, and only
  // do this if PnP solution is the bottleneck.
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  cv::Matx33d R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d wRo;
  wRo << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);

  Eigen::Matrix4d T;  // homogeneous transformation matrix
  T.topLeftCorner(3, 3) = wRo;
  T.col(3).head(3) <<
    tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0, 0, 0, 1;
  return T;
}

geometry_msgs::msg::PoseWithCovarianceStamped TagDetector::makeTagPose(
  const Eigen::Matrix4d & transform,
  const Eigen::Quaternion<double> rot_quaternion,
  const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header = header;
  //===== Position and orientation
  pose.pose.pose.position.x = transform(0, 3);
  pose.pose.pose.position.y = transform(1, 3);
  pose.pose.pose.position.z = transform(2, 3);
  pose.pose.pose.orientation.x = rot_quaternion.x();
  pose.pose.pose.orientation.y = rot_quaternion.y();
  pose.pose.pose.orientation.z = rot_quaternion.z();
  pose.pose.pose.orientation.w = rot_quaternion.w();
  return pose;
}

void TagDetector::drawDetections(cv_bridge::CvImagePtr image)
{
  for (int i = 0; i < zarray_size(detections_); i++) {
    apriltag_detection_t * det;
    zarray_get(detections_, i, &det);

    // Check if this ID is present in config/tags.yaml
    // Check if is part of a tag bundle
    int tagID = det->id;
    bool is_part_of_bundle = false;
    for (unsigned int j = 0; j < tag_bundle_descriptions_.size(); j++) {
      TagBundleDescription bundle = tag_bundle_descriptions_[j];
      if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end()) {
        is_part_of_bundle = true;
        break;
      }
    }
    // If not part of a bundle, check if defined as a standalone tag
    StandaloneTagDescription * standaloneDescription;
    if (!is_part_of_bundle &&
      !findStandaloneTagDescription(tagID, standaloneDescription, false))
    {
      // Neither a standalone tag nor part of a bundle, so this is a "rogue"
      // tag, skip it.
      continue;
    }

    // Draw tag outline with edge colors green, blue, blue, red
    // (going counter-clockwise, starting from lower-left corner in
    // tag coords). cv::Scalar(Blue, Green, Red) format for the edge
    // colors!
    line(
      image->image, cv::Point(static_cast<int>(det->p[0][0]), static_cast<int>(det->p[0][1])),
      cv::Point(static_cast<int>(det->p[1][0]), static_cast<int>(det->p[1][1])),
      cv::Scalar(0, 0xff, 0));    // green
    line(
      image->image, cv::Point(static_cast<int>(det->p[0][0]), static_cast<int>(det->p[0][1])),
      cv::Point(static_cast<int>(det->p[3][0]), static_cast<int>(det->p[3][1])),
      cv::Scalar(0, 0, 0xff));    // red
    line(
      image->image, cv::Point(static_cast<int>(det->p[1][0]), static_cast<int>(det->p[1][1])),
      cv::Point(static_cast<int>(det->p[2][0]), static_cast<int>(det->p[2][1])),
      cv::Scalar(0xff, 0, 0));    // blue
    line(
      image->image, cv::Point(static_cast<int>(det->p[2][0]), static_cast<int>(det->p[2][1])),
      cv::Point(static_cast<int>(det->p[3][0]), static_cast<int>(det->p[3][1])),
      cv::Scalar(0xff, 0, 0));    // blue

    // Print tag ID in the middle of the tag
    std::stringstream ss;
    ss << det->id;
    cv::String text = ss.str();
    int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontscale = 0.5;
    int baseline;
    cv::Size textsize = cv::getTextSize(
      text, fontface,
      fontscale, 2, &baseline);
    cv::putText(
      image->image, text,
      cv::Point(
        static_cast<int>(det->c[0] - textsize.width / 2),
        static_cast<int>(det->c[1] + textsize.height / 2)),
      fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
  }
}

// Parse standalone tag descriptions
std::map<int, StandaloneTagDescription> TagDetector::parseStandaloneTags(rclcpp::Node * node)
{
  // Create map that will be filled by the function and returned in the end
  std::map<int, StandaloneTagDescription> descriptions;
  const auto tag_names = node->declare_parameter(
    "standalone_tags.tag_names",
    std::vector<std::string>());
  // Loop through all tag descriptions
  for (const auto & name : tag_names) {
    const auto id = node->declare_parameter<int64_t>(
      "standalone_tags." + name + ".id"
    );
    const auto size = node->declare_parameter<double>(
      "standalone_tags." + name + ".size"
    );

    StandaloneTagDescription description(id, size, name);
    RCLCPP_INFO_STREAM(
      logger_,
      "Loaded tag config: " << id << ", size: " <<
        size << ", frame_name: " << name.c_str());
    // Add this tag's description to map of descriptions
    descriptions.insert(std::make_pair(id, description));
  }

  return descriptions;
}

// parse tag bundle descriptions
std::vector<TagBundleDescription> TagDetector::parseTagBundles(rclcpp::Node * node)
{
  std::vector<TagBundleDescription> descriptions;
  const auto bundle_names = node->declare_parameter(
    "tag_bundles.bundle_names",
    std::vector<std::string>());

  // Loop through all tag bundle descritions
  for (const auto & name : bundle_names) {
    TagBundleDescription bundle_i(name);
    RCLCPP_INFO(
      logger_, "Loading tag bundle '%s'", bundle_i.name().c_str());

    const auto layout_ids = node->declare_parameter(
      "tag_bundles." + name + ".layout.ids",
      std::vector<int64_t>());

    // Loop through each member tag of the bundle
    for (const auto id : layout_ids) {
      const auto prefix = "tag_bundles." + name + ".layout." + std::to_string(id) + ".";
      const auto size = node->declare_parameter<double>(prefix + "size");

      // Make sure that if this tag was specified also as standalone,
      // then the sizes match
      StandaloneTagDescription * standaloneDescription;
      if (findStandaloneTagDescription(id, standaloneDescription, false)) {
        rcpputils::assert_true(size == standaloneDescription->size());
      }

      // Get this tag's pose with respect to the bundle origin
      const auto x = node->declare_parameter(prefix + "x", 0.0);
      const auto y = node->declare_parameter(prefix + "y", 0.0);
      const auto z = node->declare_parameter(prefix + "z", 0.0);
      const auto qw = node->declare_parameter(prefix + "qw", 1.0);
      const auto qx = node->declare_parameter(prefix + "qx", 0.0);
      const auto qy = node->declare_parameter(prefix + "qy", 0.0);
      const auto qz = node->declare_parameter(prefix + "qz", 0.0);
      Eigen::Quaterniond q_tag(qw, qx, qy, qz);
      q_tag.normalize();
      Eigen::Matrix3d R_oi = q_tag.toRotationMatrix();

      // Build the rigid transform from tag_j to the bundle origin
      cv::Matx44d T_mj(R_oi(0, 0), R_oi(0, 1), R_oi(0, 2), x,
        R_oi(1, 0), R_oi(1, 1), R_oi(1, 2), y,
        R_oi(2, 0), R_oi(2, 1), R_oi(2, 2), z,
        0, 0, 0, 1);

      // Register the tag member
      bundle_i.addMemberTag(id, size, T_mj);
      RCLCPP_INFO_STREAM(
        logger_,
        "id: " << id << ", size: " << size << ", " <<
          "p = [" << x << "," << y << "," << z << "], " <<
          "q = [" << qw << "," << qx << "," << qy << "," <<
          qz << "]");
    }
    descriptions.push_back(bundle_i);
  }
  return descriptions;
}

bool TagDetector::findStandaloneTagDescription(
  int id, StandaloneTagDescription * & descriptionContainer, bool printWarning)
{
  std::map<int, StandaloneTagDescription>::iterator description_itr =
    standalone_tag_descriptions_.find(id);
  if (description_itr == standalone_tag_descriptions_.end()) {
    if (printWarning) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 10000,
        "Requested description of standalone tag ID [%d],"
        " but no description was found...", id);
    }
    return false;
  }
  descriptionContainer = &(description_itr->second);
  return true;
}

}  // namespace apriltag_ros
