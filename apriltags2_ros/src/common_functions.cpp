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
                                                run_quietly_(apriltag_getopt<bool>(pnh, "run_quietly", true)),
                                                publish_tf_(apriltag_getopt<bool>(pnh, "publish_tf", false)),
                                                rot_quaternion_previous_(0,0,0,0) {
  // parse tag descriptions specified by user (stored on ROS parameter server)
  XmlRpc::XmlRpcValue april_tag_descriptions; // stores an arbitrary XML/RPC value from the ROS parameter server
  if(!pnh.getParam("tag_descriptions", april_tag_descriptions)) {
    ROS_WARN("No april tags specified");
  } else {
    try {
      descriptions_ = parse_tag_descriptions(april_tag_descriptions);
    } catch(XmlRpc::XmlRpcException e) {
      // in case any of the asserts in parse_tag_descriptions() fail
      ROS_ERROR_STREAM("Error loading tag descriptions: " << e.getMessage().c_str());
    }
  }

  // Get frame which the sensor (camera) data is associated with
  if(!pnh.getParam("sensor_frame_id_", sensor_frame_id_)) { //
    sensor_frame_id_ = "";
  }

  // Get whether to use the P matrix (Projection/camera matrix) or K matrix (Intrinsic camera matrix) of sensor_msgs/CameraInfo message
  // true selects the P matrix (the recommended choice)
  projected_optics_ = apriltag_getopt<bool>(pnh, "projected_optics", true);

  // Set verbosity level
  if (run_quietly_) {
    // print Info, Warn, Error, Fatal messages
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
      ros::console::notifyLoggerLevelsChanged();
  } else {
    // print Debug, Info, Warn, Error, Fatal messages
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
      ros::console::notifyLoggerLevelsChanged();
  }

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

  // Run AprilTags 2 algorithm on the image
  detections_ = apriltag_detector_detect(td_, &apriltags2_image);

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

  // Set the camera frame name
  if (!sensor_frame_id_.empty())
    image->header.frame_id = sensor_frame_id_;

  // Compute the estimated translation and rotation individually for each detected tag
  AprilTagDetectionArray tag_detection_array;
  tag_detection_array.header = image->header;

  for (int i=0; i < zarray_size(detections_); i++) {

    // Get the i-th detected tag
    apriltag_detection_t *detection;
    zarray_get(detections_, i, &detection);

    // Get the description of this tag (supplied by user via ROS parameter server)
    std::map<int, TagDescription>::const_iterator description_itr = descriptions_.find(detection->id);
    if (description_itr == descriptions_.end()) {
      ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it... skipping this tag", detection->id);
      //zarray_remove
      continue;
    }
    TagDescription description = description_itr->second;
    double tag_size = description.size();

    // get estimated tag position and orientation
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
    Eigen::Matrix4d transform = getRelativeTransform(detection, tag_size, fx, fy, cx, cy);
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rot_quaternion(rot);

    // If the dot product is negative, the quaternions have opposite
    // handed-ness, Fix by reversing one quaternion.
    if (rot_quaternion.dot(rot_quaternion_previous_) < 0.0f) {
      ROS_DEBUG("Quaternion flipped. Switching sign!");
      rot_quaternion.w() *= -1;
      rot_quaternion.x() *= -1;
      rot_quaternion.y() *= -1;
      rot_quaternion.z() *= -1;
    }
    rot_quaternion_previous_ = rot_quaternion;
    
    geometry_msgs::PoseStamped tag_pose;
    tag_pose.pose.position.x = transform(0, 3);
    tag_pose.pose.position.y = transform(1, 3);
    tag_pose.pose.position.z = transform(2, 3);
    tag_pose.pose.orientation.x = rot_quaternion.x();
    tag_pose.pose.orientation.y = rot_quaternion.y();
    tag_pose.pose.orientation.z = rot_quaternion.z();
    tag_pose.pose.orientation.w = rot_quaternion.w();
    tag_pose.header = image->header;

    // Add the detection to the back of the tag detection array
    AprilTagDetection tag_detection;
    tag_detection.pose = tag_pose;
    tag_detection.id = detection->id;
    tag_detection.size = tag_size;
    tag_detection_array.detections.push_back(tag_detection);

    // print quick-reference debug info about tag detection
    if (!run_quietly_) {
      // print tag ID
      ROS_DEBUG("[tag ID %d]", detection->id);
      // print tag translation with respect to camera
      ROS_DEBUG("x: %.3f y: %.3f z: %.3f",tag_pose.pose.position.x,tag_pose.pose.position.y,tag_pose.pose.position.z);
      // print Tait-Bryan angles of tag (1. yaw about z, 2. pitch about y, 3. roll about x)
      double rad2deg = 180.0/M_PI;
      Eigen::Vector3d eulerAngles = rot.eulerAngles(2,1,0);
      // double phi = atan2(2.0*(rot_quaternion.w()*rot_quaternion.x()+rot_quaternion.y()*rot_quaternion.z()), 1.0-2.0*(rot_quaternion.x()*rot_quaternion.x()+rot_quaternion.y()*rot_quaternion.y()))*rad2deg;
      // double theta = asin(2.0*(rot_quaternion.w()*rot_quaternion.y()-rot_quaternion.z()*rot_quaternion.x()))*rad2deg;
      // double psi = atan2(2.0*(rot_quaternion.w()*rot_quaternion.z()+rot_quaternion.x()*rot_quaternion.y()), 1.0-2.0*(rot_quaternion.y()*rot_quaternion.y()+rot_quaternion.z()*rot_quaternion.z()))*rad2deg;
      double psi = eulerAngles[0]*rad2deg;
      double theta = eulerAngles[1]*rad2deg;
      double phi = eulerAngles[2]*rad2deg;
      ROS_DEBUG("yaw: %.3f pitch: %.3f roll: %.3f",psi,theta,phi);
    }

    // Publish the transform (/tf topic), making visualization in Rviz possible
    if (publish_tf_) {
      tf::Stamped<tf::Transform> tag_transform;
      tf::poseStampedMsgToTF(tag_pose, tag_transform);
      tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, description.frame_name()));
    }
  }

  return tag_detection_array;
}

Eigen::Matrix4d TagDetector::getRelativeTransform(apriltag_detection_t *detection, double tag_size, double fx, double fy, double cx, double cy) const {
  std::vector<cv::Point3f> objectPoints;
  std::vector<cv::Point2f> imagePoints;
  double s = tag_size/2;

  // set the detected tag's corner coordinates in tag coordinate frame
  objectPoints.push_back(cv::Point3f(-s,-s, 0));
  objectPoints.push_back(cv::Point3f( s,-s, 0));
  objectPoints.push_back(cv::Point3f( s, s, 0));
  objectPoints.push_back(cv::Point3f(-s, s, 0));

  // set the detected tag's corner coordinates in camera image pixel
  // coordinates
  imagePoints.push_back(cv::Point2f(detection->p[0][0], detection->p[0][1]));
  imagePoints.push_back(cv::Point2f(detection->p[1][0], detection->p[1][1]));
  imagePoints.push_back(cv::Point2f(detection->p[2][0], detection->p[2][1]));
  imagePoints.push_back(cv::Point2f(detection->p[3][0], detection->p[3][1]));

  // perform Perspective-n-Point camera pose estimation using the
  // above 3D-2D point correspondences
  // TODO extend above to N points, rather than the four corners of one tag
  cv::Mat rvec, tvec;
  cv::Matx33f cameraMatrix(fx,  0, cx,
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

// based on tag descriptions stored on the ROS parameter server, return a map from tag ID to its description
std::map<int, TagDescription> TagDetector::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions) {
  std::map<int, TagDescription> descriptions; // create map that will be filled by the function and returned in the end
  ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray); // ensure the type is correct
  for (int32_t i = 0; i < tag_descriptions.size(); i++) { // loop through all tag descriptions

    XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i]; // i-th tag description

    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct); // asser the tag description is a struct
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt); // assert type of field "id" is an int
    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble); // assert type of field "size" is a double

    int id = (int)tag_description["id"]; // tag id
    double size = (double)tag_description["size"]; // tag size (square, side length in meters)

    std::string frame_name;
    if(tag_description.hasMember("frame_id")) { // custom frame name, if such a field exists for this tag
      ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString); // asser type of field "frame_id" is a string
      frame_name = (std::string)tag_description["frame_id"];
    } else {
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id;
      frame_name = frame_name_stream.str();
    }

    TagDescription description(id, size, frame_name);
    ROS_INFO_STREAM("Loaded tag config: " << id << ", size: " << size << ", frame_name: " << frame_name.c_str());
    descriptions.insert(std::make_pair(id, description)); // add this tag's description to map of descriptions
  }

  return descriptions;
}

}
