#include "apriltags2_ros/common_functions.h"
#include <apriltags2_ros/AnalyzeSingleImage.h>

bool getRosParameter (ros::NodeHandle& pnh, std::string name, double& param)
{
  // Write parameter "name" from ROS Parameter Server into param
  // Return true if successful, false otherwise
  if (pnh.hasParam(name.c_str()))
  {
    pnh.getParam(name.c_str(), param);
    ROS_INFO_STREAM("Set camera " << name.c_str() << " = " << param);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Could not find " << name.c_str() << " parameter!");
    return false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apriltags2_ros_single_image_client");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::ServiceClient client =
      nh.serviceClient<apriltags2_ros::AnalyzeSingleImage>(
          "single_image_tag_detection");

  // Get the request parameters
  apriltags2_ros::AnalyzeSingleImage service;
  service.request.full_path_where_to_get_image =
      apriltags2_ros::getAprilTagOption<std::string>(
          pnh, "image_load_path", "");
  service.request.full_path_where_to_save_image =
      apriltags2_ros::getAprilTagOption<std::string>(
          pnh, "image_save_path", "");

  // Replicate sensors_msgs/CameraInfo message (must be up-to-date with the
  // analyzed image!)  
  service.request.camera_info.height = 480;
  service.request.camera_info.width = 752;
  service.request.camera_info.distortion_model = "plumb_bob";
  double fx, fy, cx, cy;
  if (!getRosParameter(pnh, "fx", fx))
    return 1;
  if (!getRosParameter(pnh, "fy", fy))
    return 1;
  if (!getRosParameter(pnh, "cx", cx))
    return 1;
  if (!getRosParameter(pnh, "cy", cy))
    return 1;
  // Intrinsic camera matrix for the raw (distorted) images
  service.request.camera_info.K[0] = fx;
  service.request.camera_info.K[2] = cx;
  service.request.camera_info.K[4] = fy;
  service.request.camera_info.K[5] = cy;
  service.request.camera_info.K[8] = 1.0;
  // Rectification matrix (stereo cameras only)
  service.request.camera_info.R[0] = 1.0;
  service.request.camera_info.R[4] = 1.0;
  service.request.camera_info.R[8] = 1.0;
  // Projection/camera matrix
  service.request.camera_info.P[0] = fx;
  service.request.camera_info.P[2] = cx;
  service.request.camera_info.P[5] = fy;
  service.request.camera_info.P[6] = cy;
  service.request.camera_info.P[10] = 1.0;

  // Call the service (detect tags in the image specified by the
  // image_load_path)
  if (client.call(service))
  {
    // use parameter run_quielty=false in order to have the service
    // print out the tag position and orientation
    if (service.response.tag_detections.detections.size() == 0)
    {
      ROS_WARN_STREAM("No detected tags!");
    }
  }
  else
  {
    ROS_ERROR("Failed to call service single_image_tag_detection");
    return 1;
  }

  return 0; // happy ending
}
