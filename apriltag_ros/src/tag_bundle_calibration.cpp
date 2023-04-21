#include <apriltag_ros/tag_bundle_calibration.h>

namespace apriltag_ros
{

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
  // Create a vector of constant references to the elements in tag_poses_in_master_frame
  std::vector<std::reference_wrapper<const std::pair<const int, geometry_msgs::Pose>>> sorted_tags(
      tag_poses_in_master_frame.begin(), tag_poses_in_master_frame.end());

  // Sort the vector by ascending tag ID
  std::sort(sorted_tags.begin(),
            sorted_tags.end(),
            [](const auto& a, const auto& b) { return a.get().first < b.get().first; });

  os << "standalone_tags:\n";
  os << "  [\n";
  os << "  ]\n";
  os << "tag_bundles:\n";
  os << "  [\n";
  os << "    {\n";
  os << "      name: '" << tag_bundle_name << "',\n";
  os << "      layout:\n";
  os << "        [\n";

  for (const auto& tag_entry_ref : sorted_tags)
  {
    const auto& tag_entry = tag_entry_ref.get();
    int tag_id = tag_entry.first;
    const geometry_msgs::Pose& pose = tag_entry.second;
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
TagBundleCalibrationNode::TagBundleCalibrationNode(int max_detections,
                                                   const std::string& config_file_path,
                                                   const std::string& tag_bundle_name,
                                                   int master_tag_id) :
    max_detections_(max_detections),
    config_file_path_(config_file_path),
    tag_bundle_name_(tag_bundle_name),
    master_tag_id_(master_tag_id),
    received_detections_(0)
{
  ros::NodeHandle pnh{"~"};
  size_t q_size = max_detections_ + 1; // Should allow faster than real-time stuff w rosbags
  tag_detection_sub_ = pnh.subscribe(
      "tag_detections", q_size, &TagBundleCalibrationNode::tagDetectionCallback, this);

  ROS_INFO_STREAM("Initialized tag bundle calibration node with the following parameters:\n"
                  << "  max_detections: " << max_detections_ << "\n"
                  << "  config_file_path: " << config_file_path_ << "\n"
                  << "  tag_bundle_name: " << tag_bundle_name_ << "\n"
                  << "  master_tag_id: " << master_tag_id_ << "\n");
}

void TagBundleCalibrationNode::tagDetectionCallback(
    const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
  std::unordered_map<int, geometry_msgs::Pose> curr_tag_poses_in_cam_frame;

  for (const auto& tag_detection : msg->detections)
  {
    int tag_id = tag_detection.id[0];
    double tag_size = tag_detection.size[0];
    geometry_msgs::Pose tag_pose_in_master_frame = tag_detection.pose.pose.pose;

    curr_tag_poses_in_cam_frame[tag_id] = tag_pose_in_master_frame;
    // Build up the tag size map
    if (tag_size_map_.find(tag_id) == tag_size_map_.end())
      tag_size_map_[tag_id] = tag_size;
  }

  // If the master tag is not detected, skip this detection
  if (curr_tag_poses_in_cam_frame.find(master_tag_id_) == curr_tag_poses_in_cam_frame.end())
    return;

  // Transform all tag poses to the master tag frame
  geometry_msgs::Pose master_tag_pose = curr_tag_poses_in_cam_frame[master_tag_id_];
  tf2::Transform master_tag_tf;
  tf2::fromMsg(master_tag_pose, master_tag_tf);

  // init the master tag pose if it is not already in the map
  if (tags_in_master_frame_.find(master_tag_id_) == tags_in_master_frame_.end())
    tags_in_master_frame_[master_tag_id_] = geometry_msgs::Pose{};

  for (auto& tag_pose_kv : curr_tag_poses_in_cam_frame)
  {
    if (tag_pose_kv.first == master_tag_id_)
      continue;

    tf2::Transform tag_tf;
    tf2::fromMsg(tag_pose_kv.second, tag_tf);

    tf2::Transform relative_tf = master_tag_tf.inverse() * tag_tf;
    geometry_msgs::Pose relative_pose;
    tf2::toMsg(relative_tf, relative_pose);

    int tag_id = tag_pose_kv.first;
    // If this tag is not in tags_in_master_frame_, add it.
    if (tags_in_master_frame_.find(tag_id) == tags_in_master_frame_.end())
    {
      tags_in_master_frame_[tag_id] = relative_pose;
    }
    else
    {
      // Average the current pose with the previously stored pose.
      geometry_msgs::Pose& prev_pose = tags_in_master_frame_[tag_id];

      prev_pose.position.x = (prev_pose.position.x + relative_pose.position.x) / 2;
      prev_pose.position.y = (prev_pose.position.y + relative_pose.position.y) / 2;
      prev_pose.position.z = (prev_pose.position.z + relative_pose.position.z) / 2;

      tf2::Quaternion prev_quat, relative_quat;
      tf2::fromMsg(prev_pose.orientation, prev_quat);
      tf2::fromMsg(relative_pose.orientation, relative_quat);
      prev_quat = prev_quat.slerp(relative_quat, 0.5);
      prev_pose.orientation = tf2::toMsg(prev_quat);
    }
  }

  received_detections_++;

  if (received_detections_ >= max_detections_)
  {
    std::ofstream ofs{config_file_path_};
    if (!ofs.is_open())
    {
      ROS_ERROR_STREAM("Failed to open file: " << config_file_path_);
      ros::shutdown();
      return;
    }
    detail::writeToYaml(tags_in_master_frame_, tag_size_map_, tag_bundle_name_, ofs);
    ros::shutdown();
  }
}

} // namespace apriltag_ros
