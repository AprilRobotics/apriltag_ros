#include <apriltags2_ros/continuous_detector.h>

using namespace apriltags2_msgs;
using namespace cv;
using namespace cv_bridge;
using namespace std;

int main(int argc, char **argv) {
	ros::init(argc, argv, "apriltag2 publisher");

	ros::NodeHandle nh;
	apriltags2_ros::ContinuousDetector continuousDetector(nh);

	ros::spin();
}
