//
// Created by vuwij on 02/08/18.
//

#include "../include/apriltags2_ros/tag_settings.h"

using namespace std;
using namespace ros;

namespace apriltags2_ros {

    tag_settings::tag_settings(ros::NodeHandle& nh) {
        nh.param<string>("family", family, "tag36h11");
        nh.param<int>("tag_border", border, 1);
        nh.param<int>("tag_threads", threads, 4);
        nh.param<double>("tag_decimate", decimate, 1.0);
        nh.param<double>("tag_blur", blur, 0.0);
        nh.param<int>("tag_refine_edges", refine_edges, 1);
        nh.param<int>("tag_refine_decode", refine_decode, 0);
        nh.param<int>("tag_refine_pose", refine_pose, 0);
        nh.param<int>("tag_debug", debug, 0);

        XmlRpc::XmlRpcValue standalone_tag_descriptions;
        if (!nh.getParam("standalone_tags", standalone_tag_descriptions)) {
            ROS_WARN("No april tags specified");
        } else {
            try {
                this->standalone_tag_descriptions = parseStandaloneTags(standalone_tag_descriptions);
            } catch (XmlRpc::XmlRpcException e) {
                // in case any of the asserts in parseStandaloneTags() fail
                ROS_ERROR_STREAM("Error loading standalone tag descriptions: " << e.getMessage().c_str());
            }
        }

        // parse tag bundle descriptions specified by user (stored on ROS parameter server)
        XmlRpc::XmlRpcValue tag_bundle_descriptions;
        if (!nh.getParam("tag_bundles", tag_bundle_descriptions)) {
            ROS_WARN("No tag bundles specified");
        } else {
            try {
                this->tag_bundle_descriptions = parseTagBundles(tag_bundle_descriptions);
            } catch (XmlRpc::XmlRpcException e) {
                // In case any of the asserts in parseStandaloneTags() fail
                ROS_ERROR_STREAM("Error loading tag bundle descriptions: " << e.getMessage().c_str());
            }
        }
    }

    StandaloneTagDescription* tag_settings::findStandaloneTagDescription(int id, bool printWarning) {

        if (standalone_tag_descriptions.find(id) == standalone_tag_descriptions.end()) {
            if (printWarning) {
                ROS_WARN_THROTTLE(10.0,
                                  "Requested description of standalone tag ID [%d],"
                                  " but no description was found...", id);
            }
            return NULL;
        }

        return &standalone_tag_descriptions.find(id)->second;
    }

    map<int, StandaloneTagDescription> tag_settings::parseStandaloneTags(XmlRpc::XmlRpcValue &standalone_tags) {

        // Create map that will be filled by the function and returned in the end
        std::map<int, StandaloneTagDescription> descriptions;

        // Ensure the type is correct
        ROS_ASSERT(standalone_tags.getType() == XmlRpc::XmlRpcValue::TypeArray);

        // Loop through all tag descriptions
        for (int i = 0; i < standalone_tags.size(); i++) {

            // i-th tag description
            XmlRpc::XmlRpcValue &tag_description = standalone_tags[i];

            ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
            ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
            ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

            int id = (int) tag_description["id"];           // tag id
            double size = (double) tag_description["size"]; //  (square, side length in meters)

            // Custom frame name, if such a field exists for this tag
            std::string frame_name;
            if (tag_description.hasMember("name")) {
                ROS_ASSERT(tag_description["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
                frame_name = (std::string) tag_description["name"];
            } else {
                std::stringstream frame_name_stream;
                frame_name_stream << "tag_" << id;
                frame_name = frame_name_stream.str();
            }

            StandaloneTagDescription description(id, size, frame_name);
            ROS_INFO_STREAM("Loaded tag config: " << id << ", size: " << size << ", frame_name: " << frame_name.c_str());

            descriptions.insert(std::make_pair(id, description));
        }

        return descriptions;
    }

    vector<TagBundleDescription> tag_settings::parseTagBundles(XmlRpc::XmlRpcValue &tag_bundles) {
        std::vector<TagBundleDescription> descriptions;
        ROS_ASSERT(tag_bundles.getType() == XmlRpc::XmlRpcValue::TypeArray);

        // Loop through all tag bundle descritions
        for (int i = 0; i < tag_bundles.size(); i++) {

            ROS_ASSERT(tag_bundles[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);

            XmlRpc::XmlRpcValue &bundle_description = tag_bundles[i];

            std::string bundleName;
            if (bundle_description.hasMember("name")) {
                ROS_ASSERT(bundle_description["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
                bundleName = (std::string) bundle_description["name"];
            } else {
                std::stringstream bundle_name_stream;
                bundle_name_stream << "bundle_" << i;
                bundleName = bundle_name_stream.str();
            }

            TagBundleDescription bundle_i(bundleName);
            ROS_INFO("Loading tag bundle '%s'", bundle_i.name.c_str());

            ROS_ASSERT(bundle_description["layout"].getType() == XmlRpc::XmlRpcValue::TypeArray);
            XmlRpc::XmlRpcValue &member_tags = bundle_description["layout"];

            // Loop through each member tag of the bundle
            for (int j = 0; j < member_tags.size(); j++) {
                XmlRpc::XmlRpcValue& tag = member_tags[j];

                ROS_ASSERT(member_tags[j].getType() == XmlRpc::XmlRpcValue::TypeStruct);
                ROS_ASSERT(tag["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                ROS_ASSERT(tag["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

                int id = tag["id"];
                double size = tag["size"];

                // Make sure that if this tag was specified also as standalone,
                // then the sizes match
                StandaloneTagDescription *standaloneDescription = findStandaloneTagDescription(id, false);
                ROS_ASSERT(size == standaloneDescription->size);

                // Get this tag's pose with respect to the bundle origin
                double x = xmlRpcGetDoubleWithDefault(tag, "x", 0.);
                double y = xmlRpcGetDoubleWithDefault(tag, "y", 0.);
                double z = xmlRpcGetDoubleWithDefault(tag, "z", 0.);
                double qw = xmlRpcGetDoubleWithDefault(tag, "qw", 1.);
                double qx = xmlRpcGetDoubleWithDefault(tag, "qx", 0.);
                double qy = xmlRpcGetDoubleWithDefault(tag, "qy", 0.);
                double qz = xmlRpcGetDoubleWithDefault(tag, "qz", 0.);

                Eigen::Quaterniond q_tag(qw, qx, qy, qz);
                q_tag.normalize();

                Eigen::Matrix3d R_oi = q_tag.toRotationMatrix();

                // Build the rigid transform from tag_j to the bundle origin
                cv::Matx44d T_mj(R_oi(0, 0), R_oi(0, 1), R_oi(0, 2), x, R_oi(1, 0),
                                 R_oi(1, 1), R_oi(1, 2), y, R_oi(2, 0), R_oi(2, 1),
                                 R_oi(2, 2), z, 0, 0, 0, 1);

                // Register the tag member
                bundle_i.addMemberTag(id, size, T_mj);
                ROS_INFO_STREAM(
                        " " << j << ") id: " << id << ", size: " << size << ", " << "p = [" << x << "," << y << "," << z
                            << "], " << "q = [" << qw << "," << qx << "," << qy << "," << qz << "]");
            }
            descriptions.push_back(bundle_i);
        }
        return descriptions;
    }

    double xmlRpcGetDoubleWithDefault(XmlRpc::XmlRpcValue &xmlValue, std::string field, double defaultValue)  {
        if (xmlValue.hasMember(field)) {
            ROS_ASSERT((xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    || (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt));

            if (xmlValue[field].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                return (double) xmlValue[field];
            } else {
                return xmlValue[field];
            }
        } else {
            return defaultValue;
        }
    }
}
