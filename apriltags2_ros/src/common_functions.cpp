#undef NDEBUG

#include <apriltags2_ros/common_header.h>

namespace apriltags2_ros {

TagDetector::TagDetector(ros::NodeHandle pnh) : family_(apriltag_getopt<std::string>(pnh, "tag_family", "tag36h11")),
                                                border_(apriltag_getopt<int>(pnh, "tag_border", 1)),
                                                threads_(apriltag_getopt<int>(pnh, "tag_threads", 4)),
                                                decimate_(apriltag_getopt<double>(pnh, "tag_decimate", 1.0)),
                                                blur_(apriltag_getopt<double>(pnh, "tag_blur", 0.0)),
                                                refine_edges_(apriltag_getopt<int>(pnh, "tag_refine_edges", 1)),
                                                refine_decode_(apriltag_getopt<int>(pnh, "tag_refine_decode", 0)),
                                                refine_pose_(apriltag_getopt<int>(pnh, "tag_refine_pose", 0)),
                                                debug_(apriltag_getopt<int>(pnh, "tag_debug", 0)),
                                                publish_tf_(apriltag_getopt<bool>(pnh, "publish_tf", false)) {
  // parse standalone tag descriptions specified by user (stored on ROS parameter server)
  XmlRpc::XmlRpcValue standalone_tag_descriptions; // stores an arbitrary XML/RPC value from the ROS parameter server
  if(!pnh.getParam("standalone_tags", standalone_tag_descriptions)) {
    ROS_WARN("No april tags specified");
  } else {
    try {
      standalone_tag_descriptions_ = parse_standalone_tags(standalone_tag_descriptions);
    } catch(XmlRpc::XmlRpcException e) {
      // in case any of the asserts in parse_standalone_tags() fail
      ROS_ERROR_STREAM("Error loading standalone tag descriptions: " << e.getMessage().c_str());
    }
  }

  // parse tag bundle descriptions specified by user (stored on ROS parameter server)
  XmlRpc::XmlRpcValue tag_bundle_descriptions;
  if(!pnh.getParam("tag_bundles", tag_bundle_descriptions)) {
    ROS_WARN("No tag bundles specified");
  } else {
    try {
      tag_bundle_descriptions_ = parse_tag_bundles(tag_bundle_descriptions);
    } catch(XmlRpc::XmlRpcException e) {
      // in case any of the asserts in parse_standalone_tags() fail
      ROS_ERROR_STREAM("Error loading tag bundle descriptions: " << e.getMessage().c_str());
    }
  }

  // Get whether to use the P matrix (Projection/camera matrix) or K matrix (Intrinsic camera matrix) of sensor_msgs/CameraInfo message
  // true selects the P matrix (the recommended choice)
  projected_optics_ = apriltag_getopt<bool>(pnh, "projected_optics", true);

  // Define the tag family whose tags should be searched for in the camera images
  if (family_ == "tag36h11") {
    tf_ = tag36h11_create();
  } else if (family_ == "tag36h10") {
    tf_ = tag36h10_create();
  } else if (family_ == "tag36artoolkit") {
    tf_ = tag36artoolkit_create();
  } else if (family_ == "tag25h9") {
    tf_ = tag25h9_create();
  } else if (family_ == "tag25h7") {
    tf_ = tag25h7_create();
  } else {
    ROS_WARN("Invalid tag family specified; defaulting to 36h11");
    family_ = "tag36h11";
    tf_ = tag36h11_create();
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

  // Create the apriltags variance models (result of noise statistical analysis)
  setupVarianceModels();
}

// destructor
TagDetector::~TagDetector() {
  // free memory associated with tag detector
  apriltag_detector_destroy(td_);

  // Free memory associated with the array of tag detections
  zarray_destroy(detections_);

  // free memory associated with tag family
  if (family_ == "tag36h11") {
    tag36h11_destroy(tf_);
  } else if (family_ == "tag36h10") {
    tag36h10_destroy(tf_);
  } else if (family_ == "tag36artoolkit") {
    tag36artoolkit_destroy(tf_);
  } else if (family_ == "tag25h9") {
    tag25h9_destroy(tf_);
  } else if (family_ == "tag25h7") {
    tag25h7_destroy(tf_);
  }
}

AprilTagDetectionArray TagDetector::detect_tags(const cv_bridge::CvImagePtr& image, const sensor_msgs::CameraInfoConstPtr& camera_info) {
  // Convert image to AprilTag code's format
  cv::Mat gray_image;
  cv::cvtColor(image->image, gray_image, CV_BGR2GRAY);
  image_u8_t apriltags2_image = { .width = gray_image.cols,
                                  .height = gray_image.rows,
                                  .stride = gray_image.cols,
                                  .buf = gray_image.data
  };

  // Get camera intrinsic properties
  double fx; // focal length in camera x-direction (in pixels)
  double fy; // focal length in camera y-direction (in pixels)
  double cx; // optical center x-coordinate (in pixels)
  double cy; // optical center y-coordinate (in pixels)
  if (projected_optics_) {
    // use projected focal length and principal point
    // these are the correct values
    fx = camera_info->P[0];
    fy = camera_info->P[5];
    cx = camera_info->P[2];
    cy = camera_info->P[6];
  } else {
    // use camera intrinsic focal length and principal point
    // (for backwards compatability)
    fx = camera_info->K[0];
    fy = camera_info->K[4];
    cx = camera_info->K[2];
    cy = camera_info->K[5];
  }

  // Run AprilTags 2 algorithm on the image
  detections_ = apriltag_detector_detect(td_, &apriltags2_image);

  // Restriction: any tag ID can appear at most once in the scene.
  // Thus, get all the tags visible in the scene and remove any tags with IDs of which there are multiple in the scene
  removeDuplicates();

  // Compute the estimated translation and rotation individually for each detected tag
  AprilTagDetectionArray tag_detection_array;
  std::vector<std::string > detection_names;
  tag_detection_array.header = image->header;
  std::map<std::string, std::vector<cv::Point3d > > bundleObjectPoints;
  std::map<std::string, std::vector<cv::Point2d > > bundleImagePoints;
  for (int i=0; i < zarray_size(detections_); i++) {

    // Get the i-th detected tag
    apriltag_detection_t *detection;
    zarray_get(detections_, i, &detection);

    // Bootstrap this for loop to find this tag's description amongst
    // the tag bundles. If found, add its points to the bundle's set of
    // object-image corresponding points (tag corners) for cv::solvePnP.
    // Don't yet run cv::solvePnP on the bundles, though, since we're still in
    // the process of collecting all the object-image corresponding points
    int tagID = detection->id;
    bool is_part_of_bundle = false;
    for (int j=0; j<tag_bundle_descriptions_.size(); j++) {
      // Iterate over the registered bundles
      TagBundleDescription bundle = tag_bundle_descriptions_[j];
      
      if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end()) {
        // This detected tag belongs to the j-th tag bundle (its ID was found in the bundle description)
        is_part_of_bundle = true;
        std::string bundleName = bundle.name();

        //===== Corner points in the world frame coordinates
        double s = bundle.memberSize(tagID)/2;
        addObjectPoints(s, bundle.memberT_oi(tagID), bundleObjectPoints[bundleName]);

        //===== Corner points in the image frame coordinates
        addImagePoints(detection, bundleImagePoints[bundleName]);
      }
    }

    // Find this tag's description amongst the standalone tags
    // Print warning when a tag was found that is neither part of a bundle nor standalone (thus it is a tag in the
    // environment which the user specified no description for, or Apriltags misdetected a tag (bad ID or a false
    // positive)).
    StandaloneTagDescription* standaloneDescription;
    if (!findStandaloneTagDescription(tagID, standaloneDescription, !is_part_of_bundle))
      continue;

    //=================================================================
    // The remainder of this for loop is concerned with standalone tag
    // poses!
    double tag_size = standaloneDescription->size();

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
    std::vector<cv::Point3d > standaloneTagObjectPoints;
    std::vector<cv::Point2d > standaloneTagImagePoints;
    addObjectPoints(tag_size/2, cv::Matx44d::eye(), standaloneTagObjectPoints);
    addImagePoints(detection, standaloneTagImagePoints);
    Eigen::Matrix4d transform = getRelativeTransform(standaloneTagObjectPoints, standaloneTagImagePoints, fx, fy, cx, cy);
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rot_quaternion(rot);

    // If the dot product is negative, the quaternions have opposite
    // handed-ness, Fix by reversing one quaternion.
    // TODO somehow this does not seem to work??
    if (rot_quaternion.dot(standaloneDescription->previous_quaternion_) < 0.0f) {
      flipQuaternion(rot_quaternion);
    }
    standaloneDescription->previous_quaternion_ = rot_quaternion;
    
    geometry_msgs::PoseWithCovarianceStamped tag_pose = makeTagPose(transform, rot_quaternion, image->header, (int)(standaloneDescription->size()*1000));

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

  for (int j=0; j<tag_bundle_descriptions_.size(); j++) {
    // Get bundle name
    std::string bundleName = tag_bundle_descriptions_[j].name();

    std::map<std::string, std::vector<cv::Point3d> >::iterator it = bundleObjectPoints.find(bundleName);
    if (it != bundleObjectPoints.end()) {
      // Some member tags of this bundle were detected, get the bundle's position!
      TagBundleDescription& bundle = tag_bundle_descriptions_[j];

      Eigen::Matrix4d transform = getRelativeTransform(bundleObjectPoints[bundleName], bundleImagePoints[bundleName], fx, fy, cx, cy);
      Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
      Eigen::Quaternion<double> rot_quaternion(rot);

      // If the dot product is negative, the quaternions have opposite
      // handed-ness, Fix by reversing one quaternion.
      // TODO somehow this does not seem to work??
      if (rot_quaternion.dot(bundle.previous_quaternion_) < 0.0f) {
        flipQuaternion(rot_quaternion);
      }
      bundle.previous_quaternion_ = rot_quaternion;

      geometry_msgs::PoseWithCovarianceStamped bundle_pose = makeTagPose(transform, rot_quaternion, image->header);

      // Add the detection to the back of the tag detection array
      AprilTagDetection tag_detection;
      tag_detection.pose = bundle_pose;
      tag_detection.id = bundle.bundle_ids();
      tag_detection.size = bundle.bundle_sizes();
      tag_detection_array.detections.push_back(tag_detection);
      detection_names.push_back(bundle.name());
    }
  }

  // If set, publish the transform /tf topic
  if (publish_tf_) {
    for (int i=0; i<tag_detection_array.detections.size(); i++) {
      geometry_msgs::PoseStamped pose;
      pose.pose = tag_detection_array.detections[i].pose.pose.pose;
      pose.header = tag_detection_array.detections[i].pose.header;
      tf::Stamped<tf::Transform> tag_transform;
      tf::poseStampedMsgToTF(pose, tag_transform);
      tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, "camera", detection_names[i]));
    }
  }

  return tag_detection_array;
}

int TagDetector::idComparison(const void* first, const void* second)
{
  int id1 = ((apriltag_detection_t*) first)->id;
  int id2 = ((apriltag_detection_t*) second)->id;
  return (id1 < id2) ? -1 : ((id1 == id2) ? 0 : 1);
}

void TagDetector::removeDuplicates()
{
  zarray_sort(detections_, &idComparison);
  int count = 0;
  bool duplicate_detected = false;
  while (true)
  {
    if (count >= zarray_size(detections_)-1)
    {
      break;
    }
    apriltag_detection_t *detection;
    zarray_get(detections_, count, &detection);
    int id_current = detection->id;
    zarray_get(detections_, count+1, &detection);
    int id_next = detection->id;
    if (id_current == id_next || (id_current != id_next && duplicate_detected))
    {
      duplicate_detected = true;
      // Remove the current tag detection from detections array
      int shuffle = 0;
      zarray_remove_index(detections_, count, shuffle);
      if (id_current != id_next)
      {
        ROS_WARN_STREAM("Pruning tag with ID " << id_current << " because this ID appears more than once in the image.");
        duplicate_detected = false; // Reset
      }
      continue;
    }
    count++;
  }
}

void TagDetector::addObjectPoints(double s, cv::Matx44d T_oi, std::vector<cv::Point3d >& objectPoints) const {
  // Add to vector the tag corners in the world coordinates (ideal)
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d(-s,-s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d( s,-s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d( s, s, 0, 1));
  objectPoints.push_back(T_oi.get_minor<3, 4>(0, 0)*cv::Vec4d(-s, s, 0, 1));
}

void TagDetector::addImagePoints(apriltag_detection_t *detection, std::vector<cv::Point2d >& imagePoints) const {
  // Add to vector the detected tag corners (going from bottom left to
  // top left in counterclockwise fashion) in the image (pixel)
  // coordinates
  imagePoints.push_back(cv::Point2d(detection->p[0][0], detection->p[0][1]));
  imagePoints.push_back(cv::Point2d(detection->p[1][0], detection->p[1][1]));
  imagePoints.push_back(cv::Point2d(detection->p[2][0], detection->p[2][1]));
  imagePoints.push_back(cv::Point2d(detection->p[3][0], detection->p[3][1]));
}

Eigen::Matrix4d TagDetector::getRelativeTransform(std::vector<cv::Point3d > objectPoints, std::vector<cv::Point2d > imagePoints, double fx, double fy, double cx, double cy) const {
  // perform Perspective-n-Point camera pose estimation using the
  // above 3D-2D point correspondences
  cv::Mat rvec, tvec;
  cv::Matx33d cameraMatrix(fx,  0, cx,
                           0,  fy, cy,
                           0,   0,  1);
  cv::Vec4f distCoeffs(0,0,0,0); // distortion coefficients
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
  cv::Matx33d R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix3d wRo;
  wRo << R(0,0), R(0,1), R(0,2), R(1,0), R(1,1), R(1,2), R(2,0), R(2,1), R(2,2);

  Eigen::Matrix4d T; // homogeneous transformation matrix
  T.topLeftCorner(3, 3) = wRo;
  T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0,0,0,1;

  return T;
}

void TagDetector::flipQuaternion(Eigen::Quaternion<double>& q) {
  ROS_DEBUG("Quaternion flipped. Switching sign!");
  q.w() *= -1;
  q.x() *= -1;
  q.y() *= -1;
  q.z() *= -1;
}

geometry_msgs::PoseWithCovarianceStamped TagDetector::makeTagPose(const Eigen::Matrix4d& transform, const Eigen::Quaternion<double> rot_quaternion, const std_msgs::Header& header, int size_mm) {
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header = header;
  //===== Position and orientation
  pose.pose.pose.position.x    = transform(0, 3);
  pose.pose.pose.position.y    = transform(1, 3);
  pose.pose.pose.position.z    = transform(2, 3);
  pose.pose.pose.orientation.x = rot_quaternion.x();
  pose.pose.pose.orientation.y = rot_quaternion.y();
  pose.pose.pose.orientation.z = rot_quaternion.z();
  pose.pose.pose.orientation.w = rot_quaternion.w();
  //===== Covariance
  if (size_mm) {
    // Compute variance only if the size_mm argument is passed (it defaults to 0, which lets us not compute the
    // covariance for the tag bundles, for example, for which it's not yet clear how to compute the covariance without
    // doing testing for every combination of detected tags in the bundle...)
    double d = std::sqrt(pose.pose.pose.position.x * pose.pose.pose.position.x +
                         pose.pose.pose.position.y * pose.pose.pose.position.y +
                         pose.pose.pose.position.z * pose.pose.pose.position.z); // camera-tag distance (estimated by AprilTags...)
    pose.pose.covariance[x]     = variance_models_[size_mm][x].variance(d);     // translation x
    pose.pose.covariance[y]     = variance_models_[size_mm][y].variance(d);     // translation y
    pose.pose.covariance[z]     = variance_models_[size_mm][z].variance(d);     // translation z
    pose.pose.covariance[psi]   = variance_models_[size_mm][psi].variance(d);   // Tait-Bryan rotation psi (yaw, about z)
    pose.pose.covariance[theta] = variance_models_[size_mm][theta].variance(d); // Tait-Bryan rotation theta (pitch, about y)
    pose.pose.covariance[phi]   = variance_models_[size_mm][phi].variance(d);   // Tait-Bryan rotation phi (roll, about x)
  }
  return pose;
}

void TagDetector::draw_detections(cv_bridge::CvImagePtr image) {
  for (int i = 0; i < zarray_size(detections_); i++) {
    apriltag_detection_t *det;
    zarray_get(detections_, i, &det);

    // draw tag outline with edge colors green, blue, blue, red
    // (going counter-clockwise, starting from lower-right corner in
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

    // print tag ID in the middle of the tag
    std::stringstream ss;
    ss << det->id;
    cv::String text = ss.str();
    int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontscale = 0.5;
    int baseline;
    cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2, &baseline);
    cv::putText(image->image, text, cv::Point((int) (det->c[0]-textsize.width/2), (int) (det->c[1]+textsize.height/2)), fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
  }
}

// Parse standalone tag descriptions
std::map<int, StandaloneTagDescription> TagDetector::parse_standalone_tags(XmlRpc::XmlRpcValue& standalone_tags) {
  std::map<int, StandaloneTagDescription> descriptions; // create map that will be filled by the function and returned in the end
  ROS_ASSERT(standalone_tags.getType() == XmlRpc::XmlRpcValue::TypeArray); // ensure the type is correct
  for (int32_t i = 0; i < standalone_tags.size(); i++) { // loop through all tag descriptions

    XmlRpc::XmlRpcValue& tag_description = standalone_tags[i]; // i-th tag description

    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct); // assert the tag description is a struct
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt); // assert type of field "id" is an int
    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble); // assert type of field "size" is a double

    int id = (int)tag_description["id"]; // tag id
    double size = (double)tag_description["size"]; // tag size (square, side length in meters)

    std::string frame_name;
    if(tag_description.hasMember("name")) { // custom frame name, if such a field exists for this tag
      ROS_ASSERT(tag_description["name"].getType() == XmlRpc::XmlRpcValue::TypeString); // assert type of field "name" is a string
      frame_name = (std::string)tag_description["name"];
    } else {
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id;
      frame_name = frame_name_stream.str();
    }

    StandaloneTagDescription description(id, size, frame_name);
    ROS_INFO_STREAM("Loaded tag config: " << id << ", size: " << size << ", frame_name: " << frame_name.c_str());
    descriptions.insert(std::make_pair(id, description)); // add this tag's description to map of descriptions
  }

  return descriptions;
}

// parse tag bundle descriptions
std::vector<TagBundleDescription > TagDetector::parse_tag_bundles(XmlRpc::XmlRpcValue& tag_bundles) {
  std::vector<TagBundleDescription > descriptions;
  ROS_ASSERT(tag_bundles.getType() == XmlRpc::XmlRpcValue::TypeArray);

  for (int32_t i=0; i<tag_bundles.size(); i++) { // loop through all tag bundle descritions

    ROS_ASSERT(tag_bundles[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    XmlRpc::XmlRpcValue& bundle_description = tag_bundles[i]; // i-th tag bundle description

    std::string bundleName;
    if (bundle_description.hasMember("name")) {
      ROS_ASSERT(bundle_description["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
      bundleName = (std::string)bundle_description["name"];
    } else {
      std::stringstream bundle_name_stream;
      bundle_name_stream << "bundle_" << i;
      bundleName = bundle_name_stream.str();
    }
    TagBundleDescription bundle_i(bundleName);
    ROS_INFO("Loading tag bundle '%s'",bundle_i.name().c_str());
    
    ROS_ASSERT(bundle_description["layout"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    XmlRpc::XmlRpcValue& member_tags = bundle_description["layout"];

    for (int32_t j=0; j<member_tags.size(); j++) { // loop through each member tag of the bundle
      
      ROS_ASSERT(member_tags[j].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue& tag = member_tags[j];

      ROS_ASSERT(tag["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
      int id = tag["id"];

      ROS_ASSERT(tag["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      double size = tag["size"];

      // Make sure that if this tag was specified also as standalone,
      // then the sizes match
      StandaloneTagDescription* standaloneDescription;
      if (findStandaloneTagDescription(id, standaloneDescription, false))
        ROS_ASSERT(size == standaloneDescription->size());
      
      // Get this tag's pose with respect to the bundle origin
      double x  = XmlRpcGetDoubleWithDefault(tag, "x", 0.);
      double y  = XmlRpcGetDoubleWithDefault(tag, "y", 0.);
      double z  = XmlRpcGetDoubleWithDefault(tag, "z", 0.);
      double qw = XmlRpcGetDoubleWithDefault(tag, "qw", 1.);
      double qx = XmlRpcGetDoubleWithDefault(tag, "qx", 0.);
      double qy = XmlRpcGetDoubleWithDefault(tag, "qy", 0.);
      double qz = XmlRpcGetDoubleWithDefault(tag, "qz", 0.);
      Eigen::Matrix3d R_oi = Eigen::Quaterniond
          (qw, qx, qy, qz).toRotationMatrix();

      // Build the rigid transform from tag_j to the bundle origin
      cv::Matx44d T_mj(R_oi(0,0), R_oi(0,1), R_oi(0,2), x,
                       R_oi(1,0), R_oi(1,1), R_oi(1,2), y,
                       R_oi(2,0), R_oi(2,1), R_oi(2,2), z,
                       0,         0,         0,         1);

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

double TagDetector::XmlRpcGetDouble(XmlRpc::XmlRpcValue& xmlValue, std::string field) const {
  ROS_ASSERT((xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
             (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
  if (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt) {
    int tmp = xmlValue[field];
    return (double)tmp;
  } else {
    return xmlValue[field];
  }
}

double TagDetector::XmlRpcGetDoubleWithDefault(XmlRpc::XmlRpcValue& xmlValue, std::string field, double defaultValue) const {
  if (xmlValue.hasMember(field)) {
    ROS_ASSERT((xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeDouble) ||
        (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt));
    if (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      int tmp = xmlValue[field];
      return (double)tmp;
    } else {
      return xmlValue[field];
    }
  } else {
    return defaultValue;
  }
}

bool TagDetector::findStandaloneTagDescription(int id, StandaloneTagDescription*& descriptionContainer, bool printWarning) {
  std::map<int, StandaloneTagDescription>::iterator description_itr = standalone_tag_descriptions_.find(id);
  if (description_itr == standalone_tag_descriptions_.end()) {
    if (printWarning)
      ROS_WARN_THROTTLE(10.0, "Requested description of standalone tag ID [%d], but no description was found...",id);
    return false;
  }
  descriptionContainer = &(description_itr->second);
  return true;
}

void TagDetector::setupVarianceModels() {
  //===== Setup variance models
  // Note 1: x, y, z in meters and psi, theta, phi in radians
  // Note 2: we enter coefficients for the **standard deviation** model, which are internally converted to variance
  //         by the VarianceModel class
  // 50mm tag
  variance_models_[50][x]      = VarianceModel(  0.0000979689 ,  0.0040006819 );
  variance_models_[50][y]      = VarianceModel(  0.0000537096 ,  0.0028771067 );
  variance_models_[50][z]      = VarianceModel( -0.0000277909 ,  0.0023923314 );
  variance_models_[50][psi]    = VarianceModel(  0.0000961662 ,  0.0234069470 );
  variance_models_[50][theta]  = VarianceModel( -0.0094082211 ,  0.1181870547 );
  variance_models_[50][phi]    = VarianceModel( -0.0122188194 ,  0.1238456932 );
  // 75mm tag
  variance_models_[75][x]      = VarianceModel( -0.0007500138 ,  0.0061656682 );
  variance_models_[75][y]      = VarianceModel(  0.0003249051 ,  0.0031488829 );
  variance_models_[75][z]      = VarianceModel( -0.0003692786 ,  0.0029935785 );
  variance_models_[75][psi]    = VarianceModel( -0.0018287831 ,  0.0248597919 );
  variance_models_[75][theta]  = VarianceModel( -0.0186382328 ,  0.1204987770 );
  variance_models_[75][phi]    = VarianceModel( -0.0183603953 ,  0.1180110859 );
  // 100mm tag
  variance_models_[100][x]     = VarianceModel( -0.0010707817 ,  0.0054477370 );
  variance_models_[100][y]     = VarianceModel( -0.0012293576 ,  0.0053599384 );
  variance_models_[100][z]     = VarianceModel( -0.0011488022 ,  0.0035798504 );
  variance_models_[100][psi]   = VarianceModel( -0.0009994809 ,  0.0182365331 );
  variance_models_[100][theta] = VarianceModel( -0.0458918748 ,  0.1182794903 );
  variance_models_[100][phi]   = VarianceModel( -0.0470683840 ,  0.1223757530 );
  // 180mm tag
  variance_models_[180][x]     = VarianceModel( -0.0010642714 ,  0.0056855851 );
  variance_models_[180][y]     = VarianceModel( -0.0020237825 ,  0.0061189816 );
  variance_models_[180][z]     = VarianceModel( -0.0005921215 ,  0.0019705435 );
  variance_models_[180][psi]   = VarianceModel(  0.0049294555 ,  0.0054276459 );
  variance_models_[180][theta] = VarianceModel(  0.0001791223 ,  0.0343592852 );
  variance_models_[180][phi]   = VarianceModel( -0.0008516336 ,  0.0363813023 );
  // 250mm tag
  variance_models_[250][x]     = VarianceModel( -0.0003442761 ,  0.0049214433 );
  variance_models_[250][y]     = VarianceModel( -0.0022600669 ,  0.0057952847 );
  variance_models_[250][z]     = VarianceModel( -0.0006121147 ,  0.0013706646 );
  variance_models_[250][psi]   = VarianceModel(  0.0049397108 ,  0.0075631516 );
  variance_models_[250][theta] = VarianceModel( -0.0079414578 ,  0.0495761328 );
  variance_models_[250][phi]   = VarianceModel( -0.0028767380 ,  0.0412820665 );
  // 400mm tag
  variance_models_[400][x]     = VarianceModel( -0.0002295080 ,  0.0046010060 );
  variance_models_[400][y]     = VarianceModel( -0.0025213187 ,  0.0065217905 );
  variance_models_[400][z]     = VarianceModel( -0.0009346325 ,  0.0019769957 );
  variance_models_[400][psi]   = VarianceModel(  0.0063640405 ,  0.0039516252 );
  variance_models_[400][theta] = VarianceModel(  0.0361821896 ,  0.0273991337 );
  variance_models_[400][phi]   = VarianceModel(  0.0308802099 ,  0.0249896997 );
}

}
