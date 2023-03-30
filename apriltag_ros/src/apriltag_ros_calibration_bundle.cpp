#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <Eigen/Dense>

using namespace apriltag_ros;
#define MASTER_ID 0

int g_frame_cnt = 0;
int input_frame_count = 0;

std::vector<apriltag_ros::AprilTagDetectionArray> detection_array_deque;

Eigen::Matrix3d quat2Rotmap(const geometry_msgs::Quaternion &ros_q)
{
    Eigen::Quaterniond q(ros_q.w, ros_q.x, ros_q.y, ros_q.z);
    return q.matrix();
}

Eigen::Matrix4d createTransform(const geometry_msgs::PoseWithCovarianceStamped &pc)
{
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    Eigen::Vector3d t(pc.pose.pose.position.x, pc.pose.pose.position.y, pc.pose.pose.position.z);
    Eigen::Quaterniond q(pc.pose.pose.orientation.w,
                         pc.pose.pose.orientation.x,
                         pc.pose.pose.orientation.y,
                         pc.pose.pose.orientation.z);
    transform.rotate(q);
    transform.pretranslate(t);
    return transform.matrix();
}

void tagDetectionCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &detection_array)
{
    // Use 500 frame detections to generate a calibration result
    if (g_frame_cnt++ > input_frame_count)
    {
        double master_size;
        std::map<int, std::vector<Eigen::Matrix4d>> rel_mats; // 某帧某个tag
        for (const AprilTagDetectionArray &detections : detection_array_deque)
        {
            auto it = std::find_if(detections.detections.begin(), detections.detections.end(), [&](const AprilTagDetection &d) {
                return d.id.front() == MASTER_ID;
            });
            // Master not detected in this detection, so this particular
            // detection is useless
            if (it == detections.detections.end())
                continue;

            // Get the master tag's rigid body transform to the camera frame
            Eigen::Matrix4d T_cm = createTransform(it->pose);
            master_size = it->size.front();

            // Get the rigid body transform of every other tag to the camera frame
            std::vector<Eigen::Matrix4d> mats;
            for (const auto &detection : detections.detections)
            {
                // Skip the master, but get its size first
                if (detection.id.front() == MASTER_ID)
                    continue;
                //We already have the rigid body transform from the master tag to the camera frame (T_cm)
                int other_id = detection.id.front();

                //Get this tag's rigid body transform to the camera frame
                Eigen::Matrix4d other_T_cm = createTransform(detection.pose);

                // Deduce this tag's rigid body transform to the master tag's frame
                auto tt = T_cm.inverse();
                Eigen::Matrix4d T_m = T_cm.inverse() * other_T_cm;

                // Save the relative position and orientation of this tag to the master tag
                rel_mats[detection.id.front()].push_back(T_m);
            }
        }

        // Compute the mean position as the initial value for the minimization problem
        auto meanTranspose = [&](const std::vector<Eigen::Matrix4d> &mats) -> Eigen::Vector3d {
            double mean_x(0), mean_y(0), mean_z(0);
            for (const auto &mat : mats)
            {
                Eigen::Affine3d a3d(mat);
                mean_x += a3d.translation().x();
                mean_y += a3d.translation().y();
                mean_z += a3d.translation().z();
            }
            mean_x /= mats.size();
            mean_y /= mats.size();
            mean_z /= mats.size();
            return Eigen::Vector3d(mean_x, mean_y, mean_z);
        };

        auto quatmult = [&](const Eigen::Quaterniond &a, const Eigen::Quaterniond &b) -> Eigen::Quaterniond {
            double w = a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z();
            double x = a.w() * b.x() + a.x() * b.w() - a.y() * b.z() + a.z() * b.y();
            double y = a.w() * b.y() + a.y() * b.w() + a.x() * b.z() - a.z() * b.x();
            double z = a.w() * b.z() - a.x() * b.y() + a.y() * b.x() + a.z() * b.w();
            return Eigen::Quaterniond(w, x, y, z);
        };

        auto meanQuaternions = [&](const std::vector<Eigen::Matrix4d> &mats) -> Eigen::Quaterniond {
            double mean_x(0), mean_y(0), mean_z(0), mean_w(0);

            for (int i = 0; i < mats.size(); ++i)
            {
                Eigen::Affine3d a4d(mats[i]);
                Eigen::Quaterniond q1(Eigen::Affine3d(mats[i]).rotation());
                for (int j = 0; j < mats.size(); ++j)
                {
                    if (i == j)
                        continue;

                    Eigen::Quaterniond q2(Eigen::Affine3d(mats[j]).rotation());
                    auto q_error = quatmult(q1.inverse(), q2);
                    double q_error_w = std::min<double>(1.0, std::max<double>(q_error.w(), -1.0));
                    if (2 * std::acos(q_error_w) >= M_PI / 2.0)
                        printf("Quaternion pair q_%d and q_%d are more than 90 degrees apart!", i, j);
                }
            }
        };

        // Compute the geometric median
        std::map<int, Eigen::Affine3d> rel_mats_median;
        for (const auto &ms : rel_mats)
        {
            // avereage transpose
            auto p_0 = meanTranspose(ms.second);
            // TODO In matlab code, it applied a optimization search algorithm to search a result
	    // In this version, use mean result only
            rel_mats_median[ms.first] = Eigen::Affine3d::Identity();
            rel_mats_median[ms.first].pretranslate(p_0);
        }

        // Compute the quaternion median
        for (const auto &ms : rel_mats)
        {
            auto q_0 = meanQuaternions(ms.second);
            Eigen::MatrixXd Q(4, ms.second.size());
            for (size_t i = 0; i < ms.second.size(); ++i)
            {
                Eigen::Affine3d a3d(ms.second[i]);
                Eigen::Quaterniond qq(a3d.rotation());
                double w = qq.w();
                double x = qq.x();
                double y = qq.y();
                double z = qq.z();
                Q.col(i) = Eigen::Vector4d(w, x, y, z);
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(Q * Q.transpose());
            Eigen::Vector4d D = eigensolver.eigenvalues();
            Eigen::Matrix4d V = eigensolver.eigenvectors();
            double max_value(-9999999);
            int imax;
            for (int j = 0; j < 4; j++)
            {
                if (D[j] > max_value)
                {
                    imax = j;
                    max_value = D[j];
                }
            }
            Eigen::Quaterniond q(V.col(imax)[0], V.col(imax)[1], V.col(imax)[2], V.col(imax)[3]);
            if (q.w() < 0)
                q.coeffs()[0] = -q.w();
            rel_mats_median[ms.first].rotate(q);
        }

        for (const std::pair<int, Eigen::Affine3d> &trans : rel_mats_median)
        {
            std::cout << "{id: " << trans.first
                      << ", size: " << 0.1077
                      << ", x: " << trans.second.translation()[0]
                      << ", y: " << trans.second.translation()[1]
                      << ", z: " << trans.second.translation()[2];
            Eigen::Quaterniond q(trans.second.rotation());
            std::cout << ", qw: " << q.w()
                      << ", qx: " << q.x()
                      << ", qy: " << q.y()
                      << ", qz: " << q.z()
                      << "}"
                      << std::endl;
        }

        g_frame_cnt = 0;
        detection_array_deque.clear();
    }
    else
    {
        detection_array_deque.emplace_back(*detection_array);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calib_bundel");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    private_nh.param("input_frame_count", input_frame_count, 100);

    ros::Subscriber dection_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, tagDetectionCallback);

    ros::spin();
}
