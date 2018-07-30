#!/bin/bash
#
# Detect AprilTags in a single image. Usage:
#
# ./analyze_image.sh <image_load_path> <image_save_path>
#

roslaunch apriltags2_ros single_image_client.launch image_load_path:="$1" image_save_path:="$2"
