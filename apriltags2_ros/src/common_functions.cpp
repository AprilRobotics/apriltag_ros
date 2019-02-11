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

#include "apriltags2_ros/common_functions.h"
#include "image_geometry/pinhole_camera_model.h"

#include "common/homography.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "tag16h5.h"

namespace apriltags2_ros
{

TagDetector::TagDetector(ros::NodeHandle pnh) : family_(getAprilTagOption<std::string>(pnh, "tag_family", "tag36h11")),
                                                border_(getAprilTagOption<int>(pnh, "tag_border", 1)),
                                                threads_(getAprilTagOption<int>(pnh, "tag_threads", 4)),
                                                decimate_(getAprilTagOption<double>(pnh, "tag_decimate", 1.0)),
                                                blur_(getAprilTagOption<double>(pnh, "tag_blur", 0.0)),
                                                refine_edges_(getAprilTagOption<int>(pnh, "tag_refine_edges", 1)),
                                                refine_decode_(getAprilTagOption<int>(pnh, "tag_refine_decode", 0)),
                                                refine_pose_(getAprilTagOption<int>(pnh, "tag_refine_pose", 0)),
                                                debug_(getAprilTagOption<int>(pnh, "tag_debug", 0)),
                                                publish_tf_(getAprilTagOption<bool>(pnh, "publish_tf", true))
{
  // Parse standalone tag descriptions specified by user (stored on ROS
  // parameter server)
  XmlRpc::XmlRpcValue standalone_tag_descriptions;
  if (!pnh.getParam("standalone_tags", standalone_tag_descriptions))
  {
    ROS_WARN("No april tags specified");
  }
  else
  {
    try
    {
      standalone_tag_descriptions_ = parseStandaloneTags(standalone_tag_descriptions);
    }
    catch (XmlRpc::XmlRpcException e)
    {
      // in case any of the asserts in parseStandaloneTags() fail
      ROS_ERROR_STREAM("Error loading standalone tag descriptions: " << e.getMessage().c_str());
    }
  }

  // parse tag bundle descriptions specified by user (stored on ROS parameter
  // server)
  XmlRpc::XmlRpcValue tag_bundle_descriptions;
  if (!pnh.getParam("tag_bundles", tag_bundle_descriptions))
  {
    ROS_WARN("No tag bundles specified");
  }
  else
  {
    try
    {
      tag_bundle_descriptions_ = parseTagBundles(tag_bundle_descriptions);
    }
    catch (XmlRpc::XmlRpcException e)
    {
      // In case any of the asserts in parseStandaloneTags() fail
      ROS_ERROR_STREAM("Error loading tag bundle descriptions: " << e.getMessage().c_str());
    }
  }

  // Optionally remove duplicate detections in scene. Defaults to removing
  if (!pnh.getParam("remove_duplicates", remove_duplicates_))
  {
    ROS_WARN("remove_duplicates parameter not provided. Defaulting to true");
    remove_duplicates_ = true;
  }

  // Define the tag family whose tags should be searched for in the camera
  // images
  if (family_ == "tag36h11")
  {
    tf_ = tag36h11_create();
  }
  else if (family_ == "tag36h10")
  {
    tf_ = tag36h10_create();
  }
  else if (family_ == "tag25h9")
  {
    tf_ = tag25h9_create();
  }
  else if (family_ == "tag25h7")
  {
    tf_ = tag25h7_create();
  }
  else if (family_ == "tag16h5")
  {
    tf_ = tag16h5_create();
  }
  else
  {
    ROS_WARN("Invalid tag family specified! Aborting");
    exit(1);
  }
  tf_->black_border = (uint32_t)border_;

  // Create the AprilTags 2 detector
  td_ = apriltag_detector_create();
  apriltag_detector_add_family(td_, tf_);
  td_->quad_decimate = (float)decimate_;
  td_->quad_sigma = (float)blur_;
  td_->nthreads = threads_;
  td_->debug = debug_;
  td_->refine_edges = refine_edges_;
  td_->refine_decode = refine_decode_;
  td_->refine_pose = refine_pose_;

  detections_ = NULL;

  // Get tf frame name to use for the camera
  if (!pnh.getParam("camera_frame", camera_frame_))
  {
    ROS_WARN_STREAM("Camera frame not specified, using 'camera'");
    camera_frame_ = "camera";
  }

  //CUTOMIZATION
  isUndistortionMapInitialized_ = false;
  undistortInputImage_ = false;
  pnh.getParam("undistortInputImage", undistortInputImage_);
  pnh.getParam("cameraDistortionModel", cameraDistortionModel_);
  world_frame_ = "world";
  target_frame_live_ = "target_live";
  target_frame_post_ = "target_post";
  pnh.getParam("world_frame", world_frame_);
  pnh.getParam("target_frame_live", target_frame_live_);
  pnh.getParam("target_frame_post", target_frame_post_);
  //CUSTOMIZATION
}

// destructor
TagDetector::~TagDetector()
{
  // free memory associated with tag detector
  apriltag_detector_destroy(td_);

  // Free memory associated with the array of tag detections
  apriltag_detections_destroy(detections_);

  // free memory associated with tag family
  if (family_ == "tag36h11")
  {
    tag36h11_destroy(tf_);
  }
  else if (family_ == "tag36h10")
  {
    tag36h10_destroy(tf_);
  }
  else if (family_ == "tag25h9")
  {
    tag25h9_destroy(tf_);
  }
  else if (family_ == "tag25h7")
  {
    tag25h7_destroy(tf_);
  }
  else if (family_ == "tag16h5")
  {
    tag16h5_destroy(tf_);
  }
}

AprilTagDetectionArray TagDetector::detectTags(const cv_bridge::CvImagePtr &image,
                                               const sensor_msgs::CameraInfoConstPtr &camera_info)
{
  // Convert image to Grayscale
  cv::Mat gray_image;
  if (image->image.channels() == 3) //CUSTOMIZATION
    cv::cvtColor(image->image, gray_image, CV_BGR2GRAY);
  else
    image->image.copyTo(gray_image);

  //Perform Input Image Undistortion and Rectification
  if (undistortInputImage_)
  {
    //Initialize Undistortion Map
    if (!isUndistortionMapInitialized_)
    {
      //Check model and create map //TODO: find a better way to do this
      if (cameraDistortionModel_ == "radtan")
        setRadtanUndistortRectifyMap(*camera_info);
      else if (cameraDistortionModel_ == "equidistant")
        setEquidistantUndistortRectifyMap(*camera_info);
      else
        ROS_ERROR("AprilTags2: Incorrect Camera Distortion Model selected, please set param cameraDistortionModel to either radtan or equidistant");

      //Set intrinsic properties for newly calculated unrectified Image from Optimal Camera Mtrix
      fx_ = optimalProjectionMat_.at<double>(0, 0); // Optimal focal length in camera x-direction [px]
      fy_ = optimalProjectionMat_.at<double>(1, 1); // Optimal focal length in camera y-direction [px]
      cx_ = optimalProjectionMat_.at<double>(0, 2); // Optimal optical center x-coordinate [px]
      cy_ = optimalProjectionMat_.at<double>(1, 2); // Optimal optical center y-coordinate [px]
    }

    //Undistort Image
    undistortRectifyImage(gray_image, undistortedImage_);
  }
  else
  {
    ROS_INFO_ONCE("AprilTags2: Input Image is NOT undistorted");
    gray_image.copyTo(undistortedImage_);

    // Get camera intrinsic properties for rectified image.
    image_geometry::PinholeCameraModel camera_model;
    camera_model.fromCameraInfo(camera_info);
    fx_ = camera_model.fx(); // focal length in camera x-direction [px]
    fy_ = camera_model.fy(); // focal length in camera y-direction [px]
    cx_ = camera_model.cx(); // optical center x-coordinate [px]
    cy_ = camera_model.cy(); // optical center y-coordinate [px]
  }

  // Convert Image to AprilTag Format
  image_u8_t apriltags2_image = {.width = undistortedImage_.cols, .height = undistortedImage_.rows, .stride = undistortedImage_.cols, .buf = undistortedImage_.data};

  // Run AprilTags 2 algorithm on the image
  if (detections_)
  {
    apriltag_detections_destroy(detections_);
    detections_ = NULL;
  }
  detections_ = apriltag_detector_detect(td_, &apriltags2_image);

  // If remove_dulpicates_ is set to true, then duplicate tags are not allowed.
  // Thus any duplicate tag IDs visible in the scene must include at least 1
  // erroneous detection. Remove any tags with duplicate IDs to ensure removal
  // of these erroneous detections
  if (remove_duplicates_)
  {
    removeDuplicates();
  }

  // Compute the estimated translation and rotation individually for each
  // detected tag
  AprilTagDetectionArray tag_detection_array;
  std::vector<std::string> detection_names;
  tag_detection_array.header = image->header;
  std::map<std::string, std::vector<cv::Point3d>> bundleObjectPoints;
  std::map<std::string, std::vector<cv::Point2d>> bundleImagePoints;
  for (int i = 0; i < zarray_size(detections_); i++)
  {
    // Get the i-th detected tag
    apriltag_detection_t *detection;
    zarray_get(detections_, i, &detection);
    int tagID = detection->id;

    // Bootstrap this for loop to find this tag's description amongst
    // the tag bundles. If found, add its points to the bundle's set of
    // object-image corresponding points (tag corners) for cv::solvePnP.
    // Don't yet run cv::solvePnP on the bundles, though, since we're still in
    // the process of collecting all the object-image corresponding points
    bool is_part_of_bundle = false;
    for (unsigned int j = 0; j < tag_bundle_descriptions_.size(); j++)
    {
      // Iterate over the registered bundles
      TagBundleDescription bundle = tag_bundle_descriptions_[j];

      if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end())
      {
        // This detected tag belongs to the j-th tag bundle (its ID was found in
        // the bundle description)
        is_part_of_bundle = true;
        std::string bundleName = bundle.name();

        //===== Corner points in the world frame coordinates
        double s = bundle.memberSize(tagID) / 2;
        addObjectPoints(s, bundle.memberT_oi(tagID), bundleObjectPoints[bundleName]);

        //===== Corner points in the image frame coordinates
        addImagePoints(detection, bundleImagePoints[bundleName]);
      }
    }

    // Find this tag's description amongst the standalone tags
    // Print warning when a tag was found that is neither part of a
    // bundle nor standalone (thus it is a tag in the environment
    // which the user specified no description for, or Apriltags
    // misdetected a tag (bad ID or a false positive)).
    StandaloneTagDescription *standaloneDescription;
    if (!findStandaloneTagDescription(tagID, standaloneDescription, !is_part_of_bundle))
      continue; //If tag not found loop to next tag

    //=================================================================
    // The remainder of this for loop is concerned with standalone tag poses!

    // Get estimated tag pose in the camera frame.
    //
    // Note on frames:
    // The raw AprilTags 2 uses the following frames:
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
    // TODO solvePnP[Ransac] better?
    double tag_size = standaloneDescription->size();
    std::vector<cv::Point3d> standaloneTagObjectPoints;
    addObjectPoints(tag_size / 2.0, cv::Matx44d::eye(), standaloneTagObjectPoints);
    std::vector<cv::Point2d> standaloneTagImagePoints;
    addImagePoints(detection, standaloneTagImagePoints);

    Eigen::Matrix4d transform = getRelativeTransform(standaloneTagObjectPoints,
                                                     standaloneTagImagePoints,
                                                     fx_, fy_, cx_, cy_);
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rot_quaternion(rot);

    geometry_msgs::PoseWithCovarianceStamped tag_pose = makeTagPose(transform, rot_quaternion, image->header); //Pose of Tag In Camera Frame

    // Add the detection to the back of the tag detection array
    AprilTagDetection tag_detection;
    tag_detection.pose = tag_pose;
    tag_detection.id.push_back(detection->id);
    tag_detection.size.push_back(tag_size);
    tag_detection_array.detections.push_back(tag_detection);
    detection_names.push_back(standaloneDescription->frame_name());
  }

  //=================================================================
  // Estimate bundle origin pose for each bundle in which at least one
  // member tag was detected
  for (unsigned int j = 0; j < tag_bundle_descriptions_.size(); j++)
  {
    // Get bundle name
    std::string bundleName = tag_bundle_descriptions_[j].name();

    std::map<std::string,
             std::vector<cv::Point3d>>::iterator it =
        bundleObjectPoints.find(bundleName);
    if (it != bundleObjectPoints.end())
    {
      // Some member tags of this bundle were detected, get the bundle's
      // position!
      TagBundleDescription &bundle = tag_bundle_descriptions_[j];

      Eigen::Matrix4d transform =
          getRelativeTransform(bundleObjectPoints[bundleName],
                               bundleImagePoints[bundleName], fx_, fy_, cx_, cy_);
      Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
      Eigen::Quaternion<double> rot_quaternion(rot);

      geometry_msgs::PoseWithCovarianceStamped bundle_pose =
          makeTagPose(transform, rot_quaternion, image->header);

      // Add the detection to the back of the tag detection array
      AprilTagDetection tag_detection;
      tag_detection.pose = bundle_pose;
      tag_detection.id = bundle.bundleIds();
      tag_detection.size = bundle.bundleSizes();
      tag_detection_array.detections.push_back(tag_detection);
      detection_names.push_back(bundle.name());
    }
  }

  // If set, publish the transform /tf topic
  if (publish_tf_)
  {
    for (unsigned int i = 0; i < tag_detection_array.detections.size(); i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose = tag_detection_array.detections[i].pose.pose.pose;
      pose.header = tag_detection_array.detections[i].pose.header;
      tf::Stamped<tf::Transform> tag_transform;
      tf::poseStampedMsgToTF(pose, tag_transform);
      tf_boradcaster_.sendTransform(tf::StampedTransform(tag_transform,
                                                         tag_transform.stamp_,
                                                         camera_frame_,
                                                         detection_names[i]));
    }

    //To avoid tf lookup if no tag is detected
    static bool initTransform = false;
    if (tag_detection_array.detections.size() > 0)
    {
      try
      {
        tf_listener_.waitForTransform(world_frame_, target_frame_live_, ros::Time(0), ros::Duration(0.1)); //blocking call for 0.1sec
        tf_listener_.lookupTransform(world_frame_, target_frame_live_, ros::Time(0), T_worldDARPA_);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      tf_boradcaster_.sendTransform(T_worldDARPA_);
      initTransform = true;
    }
    else if (initTransform)
      tfRepublish(ros::Time::now());
  }
  return tag_detection_array;
}

//Updates tf timestamp and republishes it
void TagDetector::tfRepublish(const ros::Time &current_stamp)
{
  T_worldDARPA_.stamp_ = current_stamp;
  T_worldDARPA_.frame_id_ = world_frame_;
  T_worldDARPA_.child_frame_id_ = target_frame_post_;
  tf_boradcaster_.sendTransform(T_worldDARPA_);
}

int TagDetector::idComparison(const void *first, const void *second)
{
  int id1 = ((apriltag_detection_t *)first)->id;
  int id2 = ((apriltag_detection_t *)second)->id;
  return (id1 < id2) ? -1 : ((id1 == id2) ? 0 : 1);
}

void TagDetector::removeDuplicates()
{
  zarray_sort(detections_, &idComparison);
  int count = 0;
  bool duplicate_detected = false;
  while (true)
  {
    if (count > zarray_size(detections_) - 1)
    {
      // The entire detection set was parsed
      return;
    }
    apriltag_detection_t *detection;
    zarray_get(detections_, count, &detection);
    int id_current = detection->id;
    // Default id_next value of -1 ensures that if the last detection
    // is a duplicated tag ID, it will get removed
    int id_next = -1;
    if (count < zarray_size(detections_) - 1)
    {
      zarray_get(detections_, count + 1, &detection);
      id_next = detection->id;
    }
    if (id_current == id_next || (id_current != id_next && duplicate_detected))
    {
      duplicate_detected = true;
      // Remove the current tag detection from detections array
      int shuffle = 0;
      zarray_remove_index(detections_, count, shuffle);
      if (id_current != id_next)
      {
        ROS_WARN_STREAM("Pruning tag ID " << id_current << " because it appears more than once in the image.");
        duplicate_detected = false; // Reset
      }
      continue;
    }
    else
    {
      count++;
    }
  }
}

void TagDetector::addObjectPoints(double s, cv::Matx44d T_oi, std::vector<cv::Point3d> &objectPoints) const
{
  // Add to object point vector the tag corner coordinates in the bundle frame
  // Going counterclockwise starting from the bottom left corner
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, -s, 0, 1)); //(x,y) and origin center of tag
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, -s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, s, 0, 1));
}

void TagDetector::addImagePoints(apriltag_detection_t *detection, std::vector<cv::Point2d> &imagePoints) const
{
  // Add to image point vector the tag corners in the image frame
  // Going counterclockwise starting from the bottom left corner
  double tag_x[4] = {-1, 1, 1, -1};
  double tag_y[4] = {1, 1, -1, -1}; // Negated because AprilTag tag local
                                    // frame has y-axis pointing DOWN
                                    // while we use the tag local frame
                                    // with y-axis pointing UP
  for (int i = 0; i < 4; i++)
  {
    // Homography projection taking tag local frame coordinates to image pixels
    double im_x, im_y;
    homography_project(detection->H, tag_x[i], tag_y[i], &im_x, &im_y);
    imagePoints.push_back(cv::Point2d(im_x, im_y));
  }
}

Eigen::Matrix4d TagDetector::getRelativeTransform(std::vector<cv::Point3d> objectPoints,
                                                  std::vector<cv::Point2d> imagePoints,
                                                  double fx, double fy, double cx, double cy) const
{
  // perform Perspective-n-Point camera pose estimation using the
  // above 3D-2D point correspondences
  cv::Mat rvec, tvec;
  cv::Matx33d cameraMatrix(fx, 0, cx,
                           0, fy, cy,
                           0, 0, 1);
  cv::Vec4f distCoeffs(0, 0, 0, 0); // distortion coefficients
  // TODO Perhaps something like SOLVEPNP_EPNP would be faster? Would
  // need to first check WHAT is a bottleneck in this code, and only
  // do this if PnP solution is the bottleneck.
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  cv::Matx33d R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d wRo;
  wRo << R(0, 0), R(0, 1), R(0, 2),
      R(1, 0), R(1, 1), R(1, 2),
      R(2, 0), R(2, 1), R(2, 2);

  Eigen::Matrix4d T; // homogeneous transformation matrix
  T.topLeftCorner(3, 3) = wRo;
  T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0, 0, 0, 1;
  return T;
}

geometry_msgs::PoseWithCovarianceStamped TagDetector::makeTagPose(const Eigen::Matrix4d &transform,
                                                                  const Eigen::Quaternion<double> rot_quaternion,
                                                                  const std_msgs::Header &header)
{
  geometry_msgs::PoseWithCovarianceStamped pose;
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

// Draw Detections on Image
void TagDetector::drawDetections(cv_bridge::CvImagePtr image)
{

  //CUSTOMIZATION
  if (image->image.channels() == 3)                             //Check if original image was color otherwise ImageTransport will have error displaying image
    cv::cvtColor(undistortedImage_, image->image, CV_GRAY2BGR); //use colorized version of image that was used for detection
  else
    undistortedImage_.copyTo(image->image);
  //CUSTOMIZATION

  for (int i = 0; i < zarray_size(detections_); i++)
  {
    apriltag_detection_t *det;
    zarray_get(detections_, i, &det);

    // Check if this ID is present in config/tags.yaml
    // Check if is part of a tag bundle
    int tagID = det->id;
    bool is_part_of_bundle = false;
    for (unsigned int j = 0; j < tag_bundle_descriptions_.size(); j++)
    {
      TagBundleDescription bundle = tag_bundle_descriptions_[j];
      if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end())
      {
        is_part_of_bundle = true;
        break;
      }
    }
    // If not part of a bundle, check if defined as a standalone tag
    StandaloneTagDescription *standaloneDescription;
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
    line(image->image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
         cv::Point((int)det->p[1][0], (int)det->p[1][1]),
         cv::Scalar(0, 0xff, 0)); // green
    line(image->image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
         cv::Point((int)det->p[3][0], (int)det->p[3][1]),
         cv::Scalar(0, 0, 0xff)); // red
    line(image->image, cv::Point((int)det->p[1][0], (int)det->p[1][1]),
         cv::Point((int)det->p[2][0], (int)det->p[2][1]),
         cv::Scalar(0xff, 0, 0)); // blue
    line(image->image, cv::Point((int)det->p[2][0], (int)det->p[2][1]),
         cv::Point((int)det->p[3][0], (int)det->p[3][1]),
         cv::Scalar(0xff, 0, 0)); // blue

    // Print tag ID in the middle of the tag
    std::stringstream ss;
    ss << det->id;
    cv::String text = ss.str();
    int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontscale = 0.5;
    int baseline;
    cv::Size textsize = cv::getTextSize(text, fontface,
                                        fontscale, 2, &baseline);
    cv::putText(image->image, text,
                cv::Point((int)(det->c[0] - textsize.width / 2),
                          (int)(det->c[1] + textsize.height / 2)),
                fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
  }
}

// Parse standalone tag descriptions
std::map<int, StandaloneTagDescription> TagDetector::parseStandaloneTags(XmlRpc::XmlRpcValue &standalone_tags)
{
  // Create map that will be filled by the function and returned in the end
  std::map<int, StandaloneTagDescription> descriptions;
  // Ensure the type is correct
  ROS_ASSERT(standalone_tags.getType() == XmlRpc::XmlRpcValue::TypeArray);
  // Loop through all tag descriptions
  for (int32_t i = 0; i < standalone_tags.size(); i++)
  {
    // i-th tag description
    XmlRpc::XmlRpcValue &tag_description = standalone_tags[i];

    // Assert the tag description is a struct
    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    // Assert type of field "id" is an int
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    // Assert type of field "size" is a double
    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    int id = (int)tag_description["id"]; // tag id
    // Tag size (square, side length in meters)
    double size = (double)tag_description["size"];

    // Custom frame name, if such a field exists for this tag
    std::string frame_name;
    if (tag_description.hasMember("name"))
    {
      // Assert type of field "name" is a string
      ROS_ASSERT(tag_description["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
      frame_name = (std::string)tag_description["name"];
    }
    else
    {
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id;
      frame_name = frame_name_stream.str();
    }

    StandaloneTagDescription description(id, size, frame_name);
    ROS_WARN_STREAM("Loaded tag config: " << id << ", size: " << size << ", frame_name: " << frame_name.c_str());
    // Add this tag's description to map of descriptions
    descriptions.insert(std::make_pair(id, description));
  }

  return descriptions;
}

// parse tag bundle descriptions
std::vector<TagBundleDescription> TagDetector::parseTagBundles(XmlRpc::XmlRpcValue &tag_bundles)
{
  std::vector<TagBundleDescription> descriptions;
  ROS_ASSERT(tag_bundles.getType() == XmlRpc::XmlRpcValue::TypeArray);

  // Loop through all tag bundle descritions
  for (int32_t i = 0; i < tag_bundles.size(); i++)
  {
    ROS_ASSERT(tag_bundles[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    // i-th tag bundle description
    XmlRpc::XmlRpcValue &bundle_description = tag_bundles[i];

    std::string bundleName;
    if (bundle_description.hasMember("name"))
    {
      ROS_ASSERT(bundle_description["name"].getType() ==
                 XmlRpc::XmlRpcValue::TypeString);
      bundleName = (std::string)bundle_description["name"];
    }
    else
    {
      std::stringstream bundle_name_stream;
      bundle_name_stream << "bundle_" << i;
      bundleName = bundle_name_stream.str();
    }
    TagBundleDescription bundle_i(bundleName);
    ROS_INFO("Loading tag bundle '%s'", bundle_i.name().c_str());

    ROS_ASSERT(bundle_description["layout"].getType() ==
               XmlRpc::XmlRpcValue::TypeArray);
    XmlRpc::XmlRpcValue &member_tags = bundle_description["layout"];

    // Loop through each member tag of the bundle
    for (int32_t j = 0; j < member_tags.size(); j++)
    {
      ROS_ASSERT(member_tags[j].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue &tag = member_tags[j];

      ROS_ASSERT(tag["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      int id = tag["id"];

      ROS_ASSERT(tag["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      double size = tag["size"];

      // Make sure that if this tag was specified also as standalone,
      // then the sizes match
      StandaloneTagDescription *standaloneDescription;
      if (findStandaloneTagDescription(id, standaloneDescription, false))
      {
        ROS_ASSERT(size == standaloneDescription->size());
      }

      // Get this tag's pose with respect to the bundle origin
      double x = xmlRpcGetDoubleWithDefault(tag, "x", 0.);
      double y = xmlRpcGetDoubleWithDefault(tag, "y", 0.);
      double z = xmlRpcGetDoubleWithDefault(tag, "z", 0.);
      double qw = xmlRpcGetDoubleWithDefault(tag, "qw", 1.);
      double qx = xmlRpcGetDoubleWithDefault(tag, "qx", 0.);
      double qy = xmlRpcGetDoubleWithDefault(tag, "qy", 0.);
      double qz = xmlRpcGetDoubleWithDefault(tag, "qz", 0.);
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
      ROS_INFO_STREAM(" " << j << ") id: " << id << ", size: " << size << ", "
                          << "p = [" << x << "," << y << "," << z << "], "
                          << "q = [" << qw << "," << qx << "," << qy << ","
                          << qz << "]");
    }
    descriptions.push_back(bundle_i);
  }
  return descriptions;
}

//Check if detected tag id is in the tags.yaml, if not found(probably false positive) then print warning
bool TagDetector::findStandaloneTagDescription(int id, StandaloneTagDescription *&descriptionContainer, bool printWarning)
{
  //Check Tag ID
  std::map<int, StandaloneTagDescription>::iterator description_itr = standalone_tag_descriptions_.find(id);

  //Tag not found
  if (description_itr == standalone_tag_descriptions_.end())
  {
    if (printWarning)
    {
      ROS_WARN_THROTTLE(10.0, "Requested description of standalone tag ID [%d], but no description was found...", id);
    }
    return false;
  }

  //Tag Found
  descriptionContainer = &(description_itr->second);
  return true;
}

double TagDetector::xmlRpcGetDouble(XmlRpc::XmlRpcValue &xmlValue, std::string field) const
{
  ROS_ASSERT((xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
             (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
  if (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    int tmp = xmlValue[field];
    return (double)tmp;
  }
  else
  {
    return xmlValue[field];
  }
}

double TagDetector::xmlRpcGetDoubleWithDefault(XmlRpc::XmlRpcValue &xmlValue, std::string field, double defaultValue) const
{
  if (xmlValue.hasMember(field))
  {
    ROS_ASSERT((xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
               (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
    if (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      int tmp = xmlValue[field];
      return (double)tmp;
    }
    else
    {
      return xmlValue[field];
    }
  }
  else
  {
    return defaultValue;
  }
}

//CUSTOMIZATION
//Image undistortion and rectification
void TagDetector::undistortRectifyImage(const cv::Mat &original_img, cv::Mat &rectified_img)
{
  //Image Rectification
  if (isUndistortionMapInitialized_)
  {
    //std::clock_t rect_t = std::clock();
    cv::remap(original_img, rectified_img, undistortMap1_, undistortMap2_, cv::INTER_LINEAR); //INTER_LINEAR(map:CV_16SC2, nninterpolation=false, avg:5.7ms) or INTER_NEAREST(map:CV_16SC2, nninterpolation=true, 1~2ms) or INTER_LINEAR(map:CV_32FC2, nninterpolation=false, avg:9.2ms)
    //std::cout << "Timing - Rectification Re-map: " << float(std::clock() - rect_t) / CLOCKS_PER_SEC * 1000.0 << "ms\n";
  }
  else
    std::cout << "-----------------Undistort-Rectification Map not initialized-----------------\n";
}

//RADTAN - Undistortion and Rectification Map Initialization
void TagDetector::setRadtanUndistortRectifyMap(sensor_msgs::CameraInfo ros_cam_param)
{
  std::cout << "-----------------RADTAN Undistort-Rectification Map initialized-----------------\n";
  //Check if Rectifcation Map is available
  if (!isUndistortionMapInitialized_)
  {
    //Get Camera Parameters
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, &ros_cam_param.K[0]);
    cv::Mat distCoeffs = cv::Mat(1, 5, CV_64F, &ros_cam_param.D[0]); //5 params - k1, k2, p1, p2, k3
    std::cout << "Camera Martix:\n"
              << cameraMatrix << std::endl;
    std::cout << "Camera Distortion Coefficients: " << distCoeffs << std::endl;

    //Optimal New Camera Matrices (Projection Matrix)
    optimalProjectionMat_ = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(ros_cam_param.width, ros_cam_param.height), 0.0);
    std::cout << "Projection Matrix:\n"
              << optimalProjectionMat_ << std::endl;

    //Create Rectification and Undistortion Maps
    cv::Mat tmpRmap1, tmpRmap2;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::noArray(), optimalProjectionMat_, cv::Size(ros_cam_param.width, ros_cam_param.height), CV_32FC1, tmpRmap1, tmpRmap2);
    cv::convertMaps(tmpRmap1, tmpRmap2, undistortMap1_, undistortMap2_, CV_16SC2, false); //INTER_NEAREST=true, INTER_LINEAR=false

    //Initialized
    isUndistortionMapInitialized_ = true;
    std::cout << "-----------------RADTAN Undistort-Rectification Map initialized-----------------\n";
  }
  else
    std::cout << "-----------------Undistort-Rectification Map ALREADY initialized-----------------\n";
}

//FISHEYE - Undistortion and Rectification Map Initialization
void TagDetector::setEquidistantUndistortRectifyMap(sensor_msgs::CameraInfo ros_cam_param)
{
  //Check if Rectifcation Map is available
  if (!isUndistortionMapInitialized_)
  {
    std::cout << "-----------------EQUIDISTANT Undistort-Rectification Map initialized-----------------\n";
    //Get Camera Parameters
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, &ros_cam_param.K[0]);
    cv::Mat distCoeffs = cv::Mat(1, 4, CV_64F, &ros_cam_param.D[0]); //4 params - k1, k2, k3, k4

    std::cout
        << "Camera Martix:\n"
        << cameraMatrix << std::endl;
    std::cout << "Camera Distortion Coefficients: " << distCoeffs << std::endl;

    //Optimal New Camera Matrices (Projection Matrix) //NOTE: Needed for fisheye calibration otherwise image is black
    cv::Mat R = cv::Mat::eye(3, 3, CV_32FC1);
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, cv::Size(ros_cam_param.width, ros_cam_param.height), R, optimalProjectionMat_);
    std::cout << "Equi-Distant Model - Projection Matrix:\n"
              << optimalProjectionMat_ << std::endl;

    //Create Rectification and Undistortion Maps
    cv::Mat tmpMap1, tmpMap2;
    cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::noArray(), optimalProjectionMat_, cv::Size(ros_cam_param.width, ros_cam_param.height), CV_32FC1, tmpMap1, tmpMap2);
    cv::convertMaps(tmpMap1, tmpMap2, undistortMap1_, undistortMap2_, CV_16SC2, false);

    //Initialized
    isUndistortionMapInitialized_ = true;
    std::cout << "-----------------EQUIDISTANT Undistort-Rectification Map initialized-----------------\n";
  }
  else
    std::cout << "-----------------Undistort-Rectification Map ALREADY initialized-----------------\n";
}
//CUSTOMIZATION

} // namespace apriltags2_ros
