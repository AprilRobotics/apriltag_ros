#ifndef APRILTAGS2_ROS_STANDALONE_TAG_DESCRIPTION_H
#define APRILTAGS2_ROS_STANDALONE_TAG_DESCRIPTION_H

using namespace std;

namespace apriltags2_ros {

class StandaloneTagDescription {
public:
	int id_;
	double size_;
	string frame_name_;

	StandaloneTagDescription(int id, double size, std::string &frame_name) :
			id_(id), size_(size), frame_name_(frame_name) {
	}
};

}

#endif
