//
// Created by vuwij on 02/08/18.
//

#ifndef PROJECT_TAG_SETTINGS_H
#define PROJECT_TAG_SETTINGS_H

#include <string>
#include <map>
#include <vector>
#include <ros/ros.h>
#include <XmlRpcException.h>

#include "standalone_tag_description.h"
#include "tag_bundle_description.h"

using namespace std;

namespace apriltags2_ros {

    class tag_settings {
    public:
        string family;
        int border;
        int threads;
        double decimate;
        double blur;
        int refine_edges;
        int refine_decode;
        int refine_pose;
        int debug;

        tag_settings(ros::NodeHandle& nh);

        // Other members
        map<int, StandaloneTagDescription> standalone_tag_descriptions;
        vector<TagBundleDescription> tag_bundle_descriptions;

        StandaloneTagDescription* findStandaloneTagDescription(int id, bool printWarning);

        // Parsing standalone tags
        map<int, StandaloneTagDescription> parseStandaloneTags(XmlRpc::XmlRpcValue &standalone_tags);

        // Parsing Tag bundle descriptions
        vector<TagBundleDescription> parseTagBundles(XmlRpc::XmlRpcValue &tag_bundles);

    };

    double xmlRpcGetDoubleWithDefault(XmlRpc::XmlRpcValue &xmlValue, string field, double defaultValue);

}
#endif //PROJECT_TAG_SETTINGS_H
