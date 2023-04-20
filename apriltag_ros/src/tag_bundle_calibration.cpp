#include <apriltag_ros/tag_bundle_calibration.h>

namespace apriltag_ros
{
void generateTagBundleConfig(
    std::ostream& os,
    const std::unordered_map<int, std::vector<AprilTagDetection>>& tag_buffer_map,
    const std::string tag_bundle_name,
    const int master_tag_id)
{
  if (tag_buffer_map.find(master_tag_id) == tag_buffer_map.end())
  {
    throw std::runtime_error("Master tag ID has not been observed");
  }

  // Calculating the average pose for each tag
  std::unordered_map<int, geometry_msgs::Pose> average_pose_map;
  std::unordered_map<int, double> tag_size_map;
  for (const auto& [tag_id, detections] : tag_buffer_map)
  {
    // Extracting the poses from the detections
    std::vector<geometry_msgs::Pose> poses;
    poses.reserve(detections.size());
    for (const auto& detection : detections)
    {
      // Ignore tag bundle detections in the tag bundle calibration routine
      if (detection.id.size() > 1)
        continue;
      poses.push_back(detection.pose.pose.pose);
    }

    // Averaging the poses
    average_pose_map[tag_id] = detail::averagePoses(poses);
    tag_size_map[tag_id] = detections[0].size[0];
  }

  // Get the master tag's pose
  const geometry_msgs::Pose& master_pose = average_pose_map[master_tag_id];

  // Convert the master pose to a tf2::Transform
  tf2::Transform master_tf;
  tf2::fromMsg(master_pose, master_tf);

  // Compute the tag poses in the frame of the master tag
  std::unordered_map<int, geometry_msgs::Pose> tag_poses_in_master_frame;
  tag_poses_in_master_frame[master_tag_id] = geometry_msgs::Pose();
  for (const auto& [tag_id, pose] : average_pose_map)
  {
    if (tag_id == master_tag_id)
      continue;

    // Convert the tag pose to a tf2::Transform
    tf2::Transform tag_tf;
    tf2::fromMsg(pose, tag_tf);

    // Calculate the transform from the master tag to the current tag
    tf2::Transform transform_in_master_frame = master_tf.inverse() * tag_tf;

    // Convert the transform back to a geometry_msgs::Pose and store it
    tag_poses_in_master_frame[tag_id] = geometry_msgs::Pose();
    tf2::toMsg(transform_in_master_frame, tag_poses_in_master_frame[tag_id]);
  }

  // Pass the tag map and the output stream to the writeToYaml function
  detail::writeToYaml(tag_poses_in_master_frame, tag_size_map, tag_bundle_name, os);
}

namespace detail
{
geometry_msgs::Pose averagePoses(const std::vector<geometry_msgs::Pose>& poses)
{
  geometry_msgs::Pose ret;

  if (poses.empty())
    return ret;

  // Averaging positions
  double sz = static_cast<double>(poses.size());
  geometry_msgs::Point init_position;
  geometry_msgs::Point avg_position =
      std::accumulate(poses.begin(),
                      poses.end(),
                      init_position,
                      [sz](geometry_msgs::Point& accum, const geometry_msgs::Pose& curr)
                      {
                        accum.x += curr.position.x / sz;
                        accum.y += curr.position.y / sz;
                        accum.z += curr.position.z / sz;
                        return accum;
                      });

  ret.position = avg_position;

  // Averaging orientations
  // https://math.stackexchange.com/questions/61146/averaging-quaternions/3435296#3435296
  tf2::Quaternion init_quat{0.0, 0.0, 0.0, 0.0};
  tf2::Quaternion first_quat, curr_quat;
  tf2::fromMsg(poses[0].orientation, first_quat);

  auto avg_quat_approx = std::accumulate(
      poses.begin(),
      poses.end(),
      init_quat,
      [&first_quat, &curr_quat](tf2::Quaternion& accum, const geometry_msgs::Pose& curr)
      {
        tf2::fromMsg(curr.orientation, curr_quat);
        double weight = curr_quat.dot(first_quat) > 0.0 ? 1.0 : -1.0;
        return accum + tf2::Quaternion(weight * curr_quat.x(),
                                       weight * curr_quat.y(),
                                       weight * curr_quat.z(),
                                       weight * curr_quat.w());
      });

  ret.orientation = tf2::toMsg(avg_quat_approx.normalize());

  return ret;
}

void writeToYaml(const std::unordered_map<int, geometry_msgs::Pose>& tag_poses_in_master_frame,
                 const std::unordered_map<int, double>& tag_size_map,
                 const std::string& tag_bundle_name,
                 std::ostream& os)
{
  os << "standalone_tags:\n";
  os << "  [\n";
  os << "  ]\n";
  os << "tag_bundles:\n";
  os << "  [\n";
  os << "    {\n";
  os << "      name: '" << tag_bundle_name << "',\n";
  os << "      layout:\n";
  os << "        [\n";

  for (const auto& [tag_id, pose] : tag_poses_in_master_frame)
  {
    double size = tag_size_map.at(tag_id);
    os << "          {id: " << tag_id << ", size: " << size << ", x: " << pose.position.x
       << ", y: " << pose.position.y << ", z: " << pose.position.z << ", qw: " << pose.orientation.w
       << ", qx: " << pose.orientation.x << ", qy: " << pose.orientation.y
       << ", qz: " << pose.orientation.z << "},\n";
  }

  os << "        ]\n";
  os << "    }\n";
  os << "  ]\n";
}

} // namespace detail

// TagBundleCalibrationNode class
TagBundleCalibrationNode::TagBundleCalibrationNode(ros::NodeHandle& nh,
                                                   int max_detections,
                                                   const std::string& config_file_path,
                                                   const std::string& tag_bundle_name,
                                                   int master_tag_id) :
    max_detections_(max_detections),
    config_file_path_(config_file_path),
    tag_bundle_name_(tag_bundle_name),
    master_tag_id_(master_tag_id),
    received_detections_(0)
{
  tag_detection_sub_ =
      nh.subscribe("tag_detection", 1, &TagBundleCalibrationNode::tagDetectionCallback, this);
}

void TagBundleCalibrationNode::tagDetectionCallback(
    const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  for (const auto& detection : msg->detections)
  {
    // Only standalone tags
    if (detection.id.size() == 1)
    {
      if (tag_buffer_map_.find(detection.id[0]) == tag_buffer_map_.end())
        tag_buffer_map_[detection.id[0]] = {};
      tag_buffer_map_[detection.id[0]].push_back(detection);
    }
  }

  received_detections_++;

  if (received_detections_ >= max_detections_)
  {
    performCalibrationAndWriteToFile();
    ros::shutdown();
  }
}

void TagBundleCalibrationNode::performCalibrationAndWriteToFile()
{
  std::ofstream ofs(config_file_path_);
  apriltag_ros::generateTagBundleConfig(ofs, tag_buffer_map_, tag_bundle_name_, master_tag_id_);
}
} // namespace apriltag_ros
