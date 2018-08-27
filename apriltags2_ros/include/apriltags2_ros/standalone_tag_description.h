#ifndef APRILTAGS2_ROS_STANDALONE_TAG_DESCRIPTION_H
#define APRILTAGS2_ROS_STANDALONE_TAG_DESCRIPTION_H

#include <string>

using namespace std;

namespace apriltags2_ros {

    class StandaloneTagDescription {
    public:
        int id;
        double size;
        string frame_name;

        StandaloneTagDescription(int id, double size, std::string &frame_name) :
                id(id), size(size), frame_name(frame_name) {
        }
    };

}

#endif
