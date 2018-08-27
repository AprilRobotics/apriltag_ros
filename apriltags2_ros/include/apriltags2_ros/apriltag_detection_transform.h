//
// Created by vuwij on 13/08/18.
//

#ifndef PROJECT_APRIL_TAG_DETECTION_TRANSFORM_H
#define PROJECT_APRIL_TAG_DETECTION_TRANSFORM_H

#include <ros/ros.h>
#include <vector>

using namespace std;

namespace apriltags2_ros {

    typedef struct transform2D {
        float delX;
        float delY;
    } Transform2D;

    typedef struct aprilTagDetectionTransform {
        int id;
        Transform2D c_delta;
        Transform2D p_delta[4];
    } AprilTagDetectionTransform;

    typedef struct aprilTagDetectionTransformArray {
        ros::Time from;
        ros::Time to;

        vector<AprilTagDetectionTransform> tagTransforms;

    } AprilTagDetectionTransformArray;

}
#endif //PROJECT_APRIL_TAG_DETECTION_TRANSFORM_H
