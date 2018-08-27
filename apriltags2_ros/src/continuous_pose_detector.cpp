#include <apriltags2_ros/apriltag_detection_transform.h>
#include <apriltags2_ros/continuous_pose_detector.h>
#include <climits>
#include <ctime>
#include <cassert>

using namespace std;

namespace apriltags2_ros {

ContinuousPoseDetector::ContinuousPoseDetector(ros::NodeHandle& nh) :
		TagDetector(nh), it(nh) {

	nh.param<bool>("publish_tag_detections_image", draw_tag_detections_image, false);
	nh.param<bool>("optical_flow_accelerated", optical_flow_accelerated, false);

	// Subscribers
	camera_image_subscriber = it.subscribeCamera("image_rect", 5, &ContinuousPoseDetector::imageCallback, this);
	april_tag_detection_array_subscriber = nh.subscribe("tag_detections", 5, &ContinuousPoseDetector::location2DCallback, this);
	tag_detections_publisher = nh.advertise<AprilTagDetectionPoseArray>("tag_pose_detections", 5);

	// Publishers
	if (draw_tag_detections_image) {
        tag_detections_image_publisher = it.advertise("tag_detections_image", 5);
    }
}

void ContinuousPoseDetector::applyTransformToDetectionArray(AprilTagDetectionArray* detectionArray, AprilTagDetectionTransformArray* transformArray, float ratio) {
    for (unsigned i = 0; i < detectionArray->detections.size(); ++i) {
        int id = detectionArray->detections[i].id;

        // Find the correct transform
        int jid = -1;
        for (unsigned j = 0; j < transformArray->tagTransforms.size(); ++j) {
            if (transformArray->tagTransforms[j].id == id) {
                jid = j;
                break;
            }
        }
        if (jid == -1) continue;

        detectionArray->detections[i].c.x += transformArray->tagTransforms[jid].c_delta.delX * ratio;
        detectionArray->detections[i].c.y += transformArray->tagTransforms[jid].c_delta.delY * ratio;

        for (unsigned c = 0; c < 4; ++c) {
            detectionArray->detections[i].p[c].x += transformArray->tagTransforms[jid].p_delta[c].delX * ratio;
            detectionArray->detections[i].p[c].y += transformArray->tagTransforms[jid].p_delta[c].delY * ratio;
        }
    }
}

void ContinuousPoseDetector::findTransformOpticalFlow(CvImageConstPtr& before, CvImageConstPtr& after, AprilTagDetectionArray* detectionArray, AprilTagDetectionTransformArray* detectionTransformArray) {
    if (detectionArray->detections.empty())
        return;

    int scale = 1; // downscales
    bool pyramid = false; // pyramid

    cv::TermCriteria termcrit(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.03);
    cv::Size subPixWinSize(10,10), winSize(31,31), pyramidWinSize(51, 51);

    uint64_t t0, t01, t1, t2, t3;

    t0 = ros::Time::now().toNSec();

    cv::Mat grayBefore, grayAfter;
    vector<cv::Mat> pyramidBefore, pyramidAfter;
    cv::cvtColor(before->image, grayBefore, cv::COLOR_BGR2GRAY );
    cv::cvtColor(after->image, grayAfter, cv::COLOR_BGR2GRAY );

    if (scale != 1) {
        cv::resize(grayBefore, grayBefore, cv::Size(grayBefore.rows / scale, grayBefore.cols / scale));
        cv::resize(grayAfter, grayAfter, cv::Size(grayAfter.rows / scale, grayAfter.cols / scale));
    }

    if (pyramid) {
        cv::buildOpticalFlowPyramid(grayBefore, pyramidBefore, pyramidWinSize, 3);
        cv::buildOpticalFlowPyramid(grayAfter, pyramidAfter, pyramidWinSize, 3);
    }

    t01 = ros::Time::now().toNSec();

    points[0].clear();
    points[1].clear();
    for (const auto &detection : detectionArray->detections) {
        points[0].push_back(cv::Point2f(detection.c.x / scale, detection.c.y / scale));
        for (unsigned i = 0; i < 4; ++i) {
            points[0].push_back(cv::Point2f(detection.p[i].x / scale, detection.p[i].y / scale));
        }
    }

    cv::cornerSubPix(grayBefore, points[0], subPixWinSize, cv::Size(-1,-1), termcrit);

    t1 = ros::Time::now().toNSec();

    vector<uchar> status;
    vector<float> err;
    if (pyramid)
        cv::calcOpticalFlowPyrLK(pyramidBefore, pyramidAfter, points[0], points[1], status, err, winSize, 3, termcrit);
    else
        cv::calcOpticalFlowPyrLK(grayBefore, grayAfter, points[0], points[1], status, err, winSize, 3, termcrit);

    t2 = ros::Time::now().toNSec();

    assert(points[0].size() == points[1].size());

    for(unsigned i = 0; i < detectionArray->detections.size(); ++i) {
        AprilTagDetectionTransform tagDetectionTransform;
        tagDetectionTransform.id = detectionArray->detections[i].id;

        tagDetectionTransform.c_delta.delX = (points[1][i * 5].x - points[0][i * 5].x) * scale;
        tagDetectionTransform.c_delta.delY = (points[1][i * 5].y - points[0][i * 5].y) * scale;
        for (unsigned j = 0; j < 4; ++j) {
            tagDetectionTransform.p_delta[j].delX = (points[1][i * 5 + j + 1].x - points[0][i * 5 + j + 1].x) * scale;
            tagDetectionTransform.p_delta[j].delY = (points[1][i * 5 + j + 1].y - points[0][i * 5 + j + 1].y) * scale;
        }
        detectionTransformArray->tagTransforms.push_back(tagDetectionTransform);
    }
    detectionTransformArray->from = before->header.stamp;
    detectionTransformArray->to = after->header.stamp;

    t3 = ros::Time::now().toNSec();

//    cout << (t01 - t0)/1000 << " " << (t1 - t01)/1000 << " " << (t2 - t1)/1000 << " " << (t3 - t2)/1000 << endl;
}

void ContinuousPoseDetector::imageCallback( const sensor_msgs::ImageConstPtr& image_rect, const sensor_msgs::CameraInfoConstPtr& camera_info) {
	// Convert ROS's sensor_msgs::Image to cv_bridge::CvImagePtr in order to run
	// AprilTags 2 on the iamge
	try {
		this->cv_image = cv_bridge::toCvShare(image_rect, sensor_msgs::image_encodings::BGR8);
		this->cv_image_drawn = cv_bridge::toCvCopy(image_rect, sensor_msgs::image_encodings::BGR8);

        this->camera_info = camera_info;
        this->imageList.push_back(this->cv_image);

	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	uint64_t t1, t2, t3, t4, t5, t6;
	t1 = ros::Time::now().toNSec();

    // 1. Find the pose transformation using optical flow
    if (!detectionArrayCorrectedList.empty() && imageList.size() >= 2) {
        CvImageConstPtr latestImage = imageList.back();
        CvImageConstPtr secondlatestImage = *(++imageList.rbegin());

        ros::Time latestImageTime = latestImage->header.stamp;
        ros::Time secondLatestImageTime = secondlatestImage->header.stamp;

//        cout << latestImageTime.sec << " " << latestImageTime.nsec << endl;
//        cout << secondLatestImageTime.sec << " " << secondLatestImageTime.nsec << endl;

        assert(latestImageTime != secondLatestImageTime);

        AprilTagDetectionArray* secondLatestPose = nullptr;

        for (auto &detectionArray : detectionArrayCorrectedList) {
            if(detectionArray.header.stamp == secondLatestImageTime) {
                secondLatestPose = &detectionArray;
                break;
            }
        }

        if (secondLatestPose != nullptr) {
            AprilTagDetectionTransformArray transformArray;
            findTransformOpticalFlow(secondlatestImage, latestImage, secondLatestPose, &transformArray);
            detectionArrayTransformList.push_back(transformArray);
        }
    }

    t2 = ros::Time::now().toNSec();

    // 2. Find the corrected pose using the past poses
    if (!detectionArrayList.empty()) {
        ros::Time image_time = cv_image->header.stamp;
        ros::Time latest_pose_time = detectionArrayList.back().header.stamp;

        AprilTagDetectionArray dArrayCurrent = detectionArrayList.back();
        dArrayCurrent.header.stamp = image_time;

        for (auto &transform : detectionArrayTransformList) {
            if (transform.from >= latest_pose_time && transform.to <= image_time) {
                applyTransformToDetectionArray(&dArrayCurrent, &transform);
            }
        }

        detectionArrayCorrectedList.push_back(dArrayCurrent);

        t3 = ros::Time::now().toNSec();

        // Draw the corrected pose on the image
        drawDetections(dArrayCurrent, cv_image_drawn);
        if (draw_tag_detections_image) {
            tag_detections_image_publisher.publish(cv_image_drawn->toImageMsg());
        }

        t4 = ros::Time::now().toNSec();

        // Find the tag poses
        AprilTagDetectionPoseArray poseArray;
        findTagPose(poseArray, dArrayCurrent);
        tag_detections_publisher.publish(poseArray);

        t5 = ros::Time::now().toNSec();
    }

    // 3. Delete old stuff from the lists
    ros::Time image_time = cv_image->header.stamp;
	ros::Duration duration(1.0);
	ros::Time too_old = image_time - duration;

    imageList.remove_if([&too_old](CvImageConstPtr& img) {return img->header.stamp < too_old; });
    detectionArrayList.remove_if([&too_old](AprilTagDetectionArray& det) {return det.header.stamp < too_old; });
    detectionArrayCorrectedList.remove_if([&too_old](AprilTagDetectionArray& det) {return det.header.stamp < too_old; });
    detectionArrayTransformList.remove_if([&too_old](AprilTagDetectionTransformArray& det) {return det.to < too_old; });

    t6 = ros::Time::now().toNSec();

    cout << "Queue sizes: " << imageList.size() << " " << detectionArrayList.size() << " " << detectionArrayCorrectedList.size() << " " << detectionArrayTransformList.size() << " | " << (t2 - t1)/1000 << " " << (t3 - t2)/1000 << " " << (t4 - t3)/1000 << " " << (t5 - t4)/1000 << " " << (t6 - t5)/1000 << endl;
}

void ContinuousPoseDetector::location2DCallback(const apriltags2_msgs::AprilTagDetectionArrayConstPtr detectionArray) {
    detectionArrayList.push_back(*detectionArray);
}

float distanceBetweenTwoPoints(float x1, float y1, float x2, float y2) {
    return (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
}

} // namespace apriltags2_ros
