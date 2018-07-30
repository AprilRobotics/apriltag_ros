#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <apriltags2_ros/continuous_detector.h>
#include <apriltags2_ros/tag_detector.h>

using namespace apriltags2_ros;
using namespace cv;

TagDetector* tag_detector;

void imageCallback(const sensor_msgs::ImageConstPtr& image_rect, const sensor_msgs::CameraInfoConstPtr& camera_info) {
	cv::Mat gray_image;

	cv_bridge::CvImagePtr cv_image;
	try {
		cv_image = cv_bridge::toCvCopy(image_rect, sensor_msgs::image_encodings::BGR8);

	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::cvtColor(cv_image->image, gray_image, CV_BGR2GRAY);
	image_u8_t apriltags2_image = {
		.width = gray_image.cols,
		.height = gray_image.rows,
		.stride = gray_image.cols,
		.buf = gray_image.data
	};

	zarray_t *detections;
	detections = apriltag_detector_detect(tag_detector->td_, &apriltags2_image);


}

int main(int argc, char **argv) {
	ros::init(argc, argv, "apriltag2 publisher");

	ros::NodeHandle nh;
	image_transport::ImageTransport it_(nh);

	tag_detector = new TagDetector(nh);

	image_transport::CameraSubscriber camera_image_subscriber = it_.subscribeCamera("image_rect", 1, &imageCallback);

	ros::spin();
}
