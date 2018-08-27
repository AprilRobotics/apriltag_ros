/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include <apriltags2_ros/tag_detector.h>
#include <apriltags2_ros/tag_bundle_description.h>
#include <apriltags2_ros/standalone_tag_description.h>
#include <apriltags2_msgs/AprilTagDetectionPoseArray.h>

#include <iostream>
#include <cstdio>
#include <ctime>

using namespace std;
using namespace cv;
using namespace cv_bridge;

namespace apriltags2_ros {

    TagDetector::TagDetector(ros::NodeHandle nh) : settings(nh) {

        nh.param<bool>("publish_tf", publish_tf, false);

        // Tag Family
        if (settings.family == "tag36h11")
            family = tag36h11_create();
        else if (settings.family == "tag36h10")
            family = tag36h10_create();
        else if (settings.family == "tag25h9")
            family = tag25h9_create();
        else if (settings.family == "tag25h7")
            family = tag25h7_create();
        else if (settings.family == "tag16h5")
            family = tag16h5_create();
        else {
            ROS_WARN("Invalid tag family specified! Aborting");
            exit(1);
        }
        family->black_border = (uint32_t) settings.border;

        // Tag Detector
        detector = apriltag_detector_create();
        detector->quad_decimate = (float) settings.decimate;
        detector->quad_sigma = (float) settings.blur;
        detector->nthreads = settings.threads;
        detector->debug = settings.debug;
        detector->refine_edges = settings.refine_edges;
        detector->refine_decode = settings.refine_decode;
        detector->refine_pose = settings.refine_pose;

        apriltag_detector_add_family(detector, family);

        // Get tf frame name to use for the camera
        if (!nh.getParam("camera_frame", camera_tf_frame)) {
            ROS_WARN_STREAM("Camera frame not specified, using 'camera'");
            camera_tf_frame = "camera";
        }
    }

    TagDetector::~TagDetector() {
        // free memory associated with tag detector
        apriltag_detector_destroy(detector);

        // free memory associated with tag family
        if (settings.family == "tag36h11")
            tag36h11_destroy(family);
        else if (settings.family == "tag36h10")
            tag36h10_destroy(family);
        else if (settings.family == "tag25h9")
            tag25h9_destroy(family);
        else if (settings.family == "tag25h7")
            tag25h7_destroy(family);
        else if (settings.family == "tag16h5")
            tag16h5_destroy(family);
    }

    void TagDetector::detectTags(AprilTagDetectionArray& detectionArray_msg, const sensor_msgs::ImageConstPtr& image_rect, const sensor_msgs::CameraInfoConstPtr& camera_info) {
        // Process the image
        CvImageConstPtr image;
        try {
            image = cv_bridge::toCvShare(image_rect, sensor_msgs::image_encodings::BGR8);

        } catch (cv_bridge::Exception& ex) {
            ROS_ERROR("cv_bridge exception: %s", ex.what());
            exit(1);
        }

        Mat gray;
        cvtColor(image->image, gray, CV_BGR2GRAY);

        image_u8_t apriltags2_image = {
                .width = gray.cols,
                .height = gray.rows,
                .stride = gray.cols,
                .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(detector, &apriltags2_image);
        removeDuplicates(detections);

        // Translate into detectionArray
        detectionArray_msg.header = image_rect->header;
        for (int i = 0; i < zarray_size(detections); i++) {
            // Get the i-th detected tag
            apriltag_detection_t *detection;
            zarray_get(detections, i, &detection);
            // Copy it into the ROS Message
            AprilTagDetection detection_msg;
            detection_msg.family.black_border = detection->family->black_border;
            detection_msg.family.ncodes = detection->family->ncodes;
            detection_msg.family.d = detection->family->d;
            detection_msg.family.h = detection->family->h;
            detection_msg.family.black_border = detection->family->black_border;
            string family_name(detection->family->name);
            detection_msg.family.name = family_name;
            detection_msg.decision_margin = detection->decision_margin;
            detection_msg.goodness = detection->goodness;
            detection_msg.hamming = detection->hamming;
            detection_msg.id = detection->id;
            detection_msg.c.x = detection->c[0];
            detection_msg.c.y = detection->c[1];


            Point2D p1, p2, p3, p4;
            p1.x = detection->p[0][0];
            p1.y = detection->p[0][1];
            p2.x = detection->p[1][0];
            p2.y = detection->p[1][1];
            p3.x = detection->p[2][0];
            p3.y = detection->p[2][1];
            p4.x = detection->p[3][0];
            p4.y = detection->p[3][1];
            detection_msg.p.push_back(p1);
            detection_msg.p.push_back(p2);
            detection_msg.p.push_back(p3);
            detection_msg.p.push_back(p4);
            detection_msg.H.ncols = detection->H->ncols;
            detection_msg.H.nrows = detection->H->nrows;
            for(int i = 0; i < detection->H->ncols * detection->H->nrows; ++i) {
                detection_msg.H.data.push_back(detection->H->data[i]);
            }
            detectionArray_msg.detections.push_back(detection_msg);
        }
        zarray_destroy(detections);
    }

    void TagDetector::findTagPose(AprilTagDetectionPoseArray& detectionPoseArray, const AprilTagDetectionArray detectionArray) {

        // Get camera intrinsic properties
        double fx = camera_info->K[0]; // focal length in camera x-direction [px]
        double fy = camera_info->K[4]; // focal length in camera y-direction [px]
        double cx = camera_info->K[2]; // optical center x-coordinate [px]
        double cy = camera_info->K[5]; // optical center y-coordinate [px]

        // Unwrap the detectionArray
        detectionPoseArray.header = detectionArray.header;

        vector<string> detection_names;

        map<string, vector<Point3d> > bundleObjectPoints;
        map<string, vector<Point2d> > bundleImagePoints;
        for (auto &detection : detectionArray.detections) {

            int id = detection.id;

            bool is_part_of_bundle = false;

            for (auto &tag_bundle_description : settings.tag_bundle_descriptions) {
                if (tag_bundle_description.id2idx.find(id) == tag_bundle_description.id2idx.end()) continue;

                is_part_of_bundle = true;
                string bundlename = tag_bundle_description.name;

                // Corner points in world frame coordinates
                double s = tag_bundle_description.memberSize(id) / 2;
                addObjectPoints(s, tag_bundle_description.memberT_oi(id), bundleObjectPoints[bundlename]);

                matd_t H;
                H.nrows = detection.H.nrows;
                H.ncols = detection.H.ncols;
                for (unsigned i = 0; i < H.ncols * H.nrows; ++i)
                    H.data[i] = detection.H.data[i];

                addImagePoints(H, bundleImagePoints[bundlename]);
            }

            // Find this tag's description among the standalone tags
            StandaloneTagDescription *standaloneTagDescription = settings.findStandaloneTagDescription(id, !is_part_of_bundle);
            if (standaloneTagDescription == NULL)
                continue;

            double tagsize = standaloneTagDescription->size;

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

            vector<cv::Point3d> standaloneTagObjectPoints;
            vector<cv::Point2d> standaloneTagImagePoints;

            addObjectPoints(tagsize / 2, cv::Matx44d::eye(), standaloneTagObjectPoints);
            matd_t H;
            H.nrows = detection.H.nrows;
            H.ncols = detection.H.ncols;
            for (unsigned i = 0; i < H.ncols * H.nrows; ++i)
                H.data[i] = detection.H.data[i];
            addImagePoints(H, standaloneTagImagePoints);

            Eigen::Matrix4d transform = getRelativeTransform(standaloneTagObjectPoints, standaloneTagImagePoints, fx, fy, cx, cy);
            Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
            Eigen::Quaternion<double> rot_quaternion(rot);

            geometry_msgs::PoseWithCovarianceStamped tag_pose = makeTagPose(transform, rot_quaternion, detectionArray.header);

            // Add the detection to the back of the tag detection array
            AprilTagDetectionPose tag_detection;
            tag_detection.pose = tag_pose;
            tag_detection.id.push_back(detection.id);
            tag_detection.size.push_back(tagsize);

            detectionPoseArray.detections.push_back(tag_detection);
            detection_names.push_back(standaloneTagDescription->frame_name);
        }

        // Estimate bundle origin pose for each bundle in which at least one member tag was detected

        for (auto bundle = settings.tag_bundle_descriptions.begin(); bundle != settings.tag_bundle_descriptions.end(); ++bundle) {
            string name = bundle->name;

            if(bundleObjectPoints.find(name) != bundleObjectPoints.end()) {

                Eigen::Matrix4d transform = getRelativeTransform(
                        bundleObjectPoints[name],
                        bundleImagePoints[name], fx, fy, cx, cy);

                Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);

                Eigen::Quaternion<double> rot_quaternion(rot);

                geometry_msgs::PoseWithCovarianceStamped bundle_pose = makeTagPose(
                        transform, rot_quaternion, detectionArray.header);

                // Add the detection to the back of the tag detection array
                AprilTagDetectionPose tag_detection;
                tag_detection.pose = bundle_pose;
                tag_detection.id = bundle->bundleIds();
                tag_detection.size = bundle->bundleSizes();

                detectionPoseArray.detections.push_back(tag_detection);
                detection_names.push_back(bundle->name);
            }
        }

        // If set, publish the transform /tf topic
        if (publish_tf) {
            for (int i = 0; i < detectionPoseArray.detections.size(); ++i) {
                geometry_msgs::PoseStamped pose;
                pose.pose = detectionPoseArray.detections[i].pose.pose.pose;
                pose.header = detectionPoseArray.detections[i].pose.header;

                tf::Stamped<tf::Transform> tag_transform;
                tf::poseStampedMsgToTF(pose, tag_transform);
                tf_pub.sendTransform(tf::StampedTransform(tag_transform,
                                                          tag_transform.stamp_, camera_tf_frame, detection_names[i]));
            }
        }
    }

    int TagDetector::idComparison(const void *first, const void *second) {
        int id1 = ((apriltag_detection_t *) first)->id;
        int id2 = ((apriltag_detection_t *) second)->id;
        return (id1 < id2) ? -1 : ((id1 == id2) ? 0 : 1);
    }

    void TagDetector::removeDuplicates(zarray_t* detections) {
        zarray_sort(detections, &idComparison);
        int idx = 0;
        bool duplicate_detected = false;

        while (true) {
            if (idx > zarray_size(detections) - 1)
                return;

            apriltag_detection_t *detection;
            zarray_get(detections, idx, &detection);

            int id_current = detection->id;

            // Default id_next value of -1 ensures that if the last detection
            // is a duplicated tag ID, it will get removed
            int id_next = -1;
            if (idx < zarray_size(detections) - 1) {
                zarray_get(detections, idx + 1, &detection);
                id_next = detection->id;
            }

            if (id_current == id_next || (id_current != id_next && duplicate_detected)) {
                duplicate_detected = true;
                // Remove the current tag detection from detections array
                int shuffle = 0;

                zarray_remove_index(detections, idx, shuffle);
                if (id_current != id_next) {
                    ROS_WARN_STREAM(
                            "Pruning tag ID " << id_current << " because it " "appears more than once in the image.");
                    duplicate_detected = false; // Reset
                }
                continue;
            } else {
                idx++;
            }
        }
    }

    void TagDetector::addObjectPoints(double s, cv::Matx44d T_oi, std::vector<cv::Point3d> &objectPoints) const {
        // Add to object point vector the tag corner coordinates in the bundle frame
        // Going counterclockwise starting from the bottom left corner
        objectPoints.emplace_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, -s, 0, 1));
        objectPoints.emplace_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, -s, 0, 1));
        objectPoints.emplace_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(s, s, 0, 1));
        objectPoints.emplace_back(T_oi.get_minor<3, 4>(0, 0) * cv::Vec4d(-s, s, 0, 1));
    }

    void TagDetector::addImagePoints(matd_t& H, std::vector<cv::Point2d> &imagePoints) const {
        // Add to image point vector the tag corners in the image frame
        // Going counterclockwise starting from the bottom left corner
        double tag_x[4] = {-1, 1, 1, -1};
        double tag_y[4] = {1, 1, -1, -1}; // Negated because AprilTag tag local
        // frame has y-axis pointing DOWN
        // while we use the tag local frame
        // with y-axis pointing UP
        for (int i = 0; i < 4; i++) {
            // Homography projection taking tag local frame coordinates to image pixels
            double im_x, im_y;
            homography_project(&H, tag_x[i], tag_y[i], &im_x, &im_y);
            imagePoints.emplace_back(im_x, im_y);
        }
    }

    Eigen::Matrix4d TagDetector::getRelativeTransform(std::vector<cv::Point3d> objectPoints, std::vector<cv::Point2d> imagePoints,
                                      double fx, double fy, double cx, double cy) const {

        // Perform Perspective-n-Point camera pose estimation using the above 3D-2D point correspondences
        cv::Mat rvec, tvec;
        cv::Matx33d cameraMatrix(fx, 0, cx, 0, fy, cy, 0, 0, 1);
        cv::Vec4f distCoeffs(0, 0, 0, 0); // distortion coefficients

        // TODO Perhaps something like SOLVEPNP_EPNP would be faster? Would
        // need to first check WHAT is a bottleneck in this code, and only
        // do this if PnP solution is the bottleneck.
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
        cv::Matx33d R;
        cv::Rodrigues(rvec, R);
        Eigen::Matrix3d wRo;

        wRo << R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2);

        Eigen::Matrix4d T; // homogeneous transformation matrix
        T.topLeftCorner(3, 3) = wRo;
        T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
        T.row(3) << 0, 0, 0, 1;
        return T;
    }

    geometry_msgs::PoseWithCovarianceStamped
    TagDetector::makeTagPose(const Eigen::Matrix4d &transform, const Eigen::Quaternion<double> rot_quaternion,
                             const std_msgs::Header &header) {

        geometry_msgs::PoseWithCovarianceStamped pose;
        pose.header = header;
        pose.pose.pose.position.x = transform(0, 3);
        pose.pose.pose.position.y = transform(1, 3);
        pose.pose.pose.position.z = transform(2, 3);
        pose.pose.pose.orientation.x = rot_quaternion.x();
        pose.pose.pose.orientation.y = rot_quaternion.y();
        pose.pose.pose.orientation.z = rot_quaternion.z();
        pose.pose.pose.orientation.w = rot_quaternion.w();
        return pose;
    }

    void TagDetector::drawDetections(const AprilTagDetectionArray& detectionArray, CvImagePtr& img) {

        for (const auto &detection : detectionArray.detections) {
            int id = detection.id;

            bool is_part_of_bundle = false;
            for (auto &tag_bundle_description : settings.tag_bundle_descriptions) {
                if (tag_bundle_description.id2idx.find(id) != tag_bundle_description.id2idx.end()) {
                    is_part_of_bundle = true;
                    break;
                }
            }

            if (!is_part_of_bundle) {
                StandaloneTagDescription* description = settings.findStandaloneTagDescription(id, false);
                if(description == nullptr)
                    continue;
            }

            // Draw tag outline with edge colors green, blue, blue, red
            // (going counter-clockwise, starting from lower-left corner in
            // tag coords). cv::Scalar(Blue, Green, Red) format for the edge
            // colors!
            line(img->image,
                    cv::Point((int) detection.p[0].x, (int) detection.p[0].y),
                    cv::Point((int) detection.p[1].x, (int) detection.p[1].y),
                    cv::Scalar(0, 0xff, 0)); // green
            line(img->image,
                    cv::Point((int) detection.p[0].x, (int) detection.p[0].y),
                    cv::Point((int) detection.p[3].x, (int) detection.p[3].y),
                    cv::Scalar(0, 0, 0xff)); // red
            line(img->image,
                    cv::Point((int) detection.p[1].x, (int) detection.p[1].y),
                    cv::Point((int) detection.p[2].x, (int) detection.p[2].y),
                    cv::Scalar(0xff, 0, 0)); // blue
            line(img->image,
                    cv::Point((int) detection.p[2].x, (int) detection.p[2].y),
                    cv::Point((int) detection.p[3].x, (int) detection.p[3].y),
                    cv::Scalar(0xff, 0, 0)); // blue

            // Print tag ID in the middle of the tag
            std::stringstream ss;
            ss << detection.id;

            cv::String text = ss.str();

            int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 0.5;
            int baseline;

            cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2, &baseline);
            cv::putText(img->image, text,
                    cv::Point((int) (detection.c.x - textsize.width / 2),
                            (int) (detection.c.y + textsize.height / 2)), fontface,
                            fontscale, cv::Scalar(0xff, 0x99, 0), 2);

        }
    }

} // namespace apriltags2_ros
