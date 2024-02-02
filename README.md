# apriltag_ros

`apriltag_ros` is a ROS2 wrapper of the [AprilTag 3 visual fiducial detector](https://april.eecs.umich.edu/software/apriltag.html).

**Authors**: Danylo Malyuta, Wolfgang Merkt

**Maintainers**: [Danylo Malyuta](mailto:danylo.malyuta@gmail.com) ([Autonomous Control Laboratory](https://www.aa.washington.edu/research/acl), University of Washington), [Wolfgang Merkt](https://github.com/wxmerkt)

## Package Overview

This package provides functionality for detecting AprilTags in both continuous image streams and individual images. It subscribes to the following default input topics, which can be remapped according to user needs:

- `/camera_rect/image_rect`: a `sensor_msgs/Image` topic containing the image, assuming it's undistorted..
- `/camera_rect/camera_info`: a `sensor_msgs/CameraInfo` topic providing the camera calibration matrix in `/camera/camera_info/K`.

The behavior of the ROS wrapper is defined by two configuration files:
- `config/tags.yaml`: Defines the tags and tag bundles to detect.
- `config/settings.yaml`: Configures the core Apriltag 3 algorithm parameters.

The package outputs the following topics:
- `/tf`: Provides the relative pose between the camera frame and each detected tag's or tag bundle's frame, specified in `tags.yaml`, using tf. This is published only if `publish_tf` is set to true in `settings.yaml`.
- `/tag_detections`: Carries the same information as `/tf`, but as a custom message containing the tag ID(s), size(s), and `geometry_msgs/PoseWithCovarianceStamped` pose information (applies in the case of tag bundles). This topic is always published.
- `/tag_detections_image`: Displays the same input image from `/camera/image_rect`, but with the detected tags highlighted. This is published only if `publish_tag_detections_image` is set to true in `config/params.yaml`.

## Quickstart

Starting with a working ROS2 installation (Humble is supported):
```
export ROS_DISTRO=humble                                        # Set this to your distro, e.g. humble
source /opt/ros/$ROS_DISTRO/setup.bash                          # Source your ROS distro 
mkdir -p ~/ros2_ws/src                                          # Make a new workspace 
cd ~/ros2_ws/src                                                # Navigate to the source space
git clone https://github.com/AprilRobotics/apriltag_ros.git     # Clone Apriltag ROS wrapper
cd ~/ros2_ws                                                    # Navigate to the workspace
rosdep install --from-paths src --ignore-src                    # Install any missing packages
colcon build --symlink-install                                  # Build all packages in the workspace
```

## Tag Size Definition

For a correct depth estimation (and hence the correct full pose) it is necessary to specify the tag size in config/tags.yaml correctly. In the [Wiki for the AprilTag Library](https://github.com/AprilRobotics/apriltag/wiki/AprilTag-User-Guide#pose-estimation)  the correct interpretation of the term "tag size" is explained. The size is defined by the length of the black/white border between the complete black and white rectangle of any tag type. Note that for apriltag3 marker families this does not in fact correspond to the outside of the marker.

Below is a visualization of the tag size (red arrow) to be specified for the most common tag classes:
![Tag Size Guide](./apriltag_ros/docs/tag_size_guide.svg)

## Tags Parameter Setup

The `config/tags.yaml` file defines the standalone tags and tag bundles that should be looked for in the input image. Rogue tags (i.e., those that are not defined in `config/tags.yaml`) will be ignored. **IMPORTANT**:

- No tag ID should appear twice with different sizes (this creates ambiguity in the detection).
- No tag ID should appear twice in the image (this creates ambiguity in the detection).
- It is fine for a tag with the same ID to be listed both in `standalone_tags` and in `tag_bundles`, as long as it has the same size.

Furthermore, make sure that you print your tags surrounded by at least a 1-bit wide white border. The core AprilTag 2 algorithm samples this surrounding white border for creating a light model over the tag surface so do not, e.g., cut or print the tags out flush with their black border.

The `tag_bundles` are filled out with the relative poses of the constituent tags to the bundle origin. Sometimes you may know these transforms (e.g., you print the bundle on a single sheet of paper where you lay out the tags in a known fashion). When you do not know these relative poses (e.g., you stick individual tags roughly on the floor/wall), perform a calibration to easily determine them.


## Nodes

1. **apriltag_ros**

    **Description:**  
    This node provides detection for a continuous image stream (e.g., video). It detects AprilTags in images produced by a pinhole camera and publishes the detections' pose relative to the camera.

    **Subscribed Topics:**  
    - `/image_rect` (`sensor_msgs/Image`): Undistorted image from the camera.
    - `/camera_info` (`sensor_msgs/CameraInfo`): Camera calibration matrix K.

    **Published Topics:**  
    - `/tag_detections` (`apriltag_ros/AprilTagDetectionArray`): Array of tag and tag bundle detections' pose relative to the camera.
    - `/tag_detections_image` (`sensor_msgs/Image`): Same as `image_rect` but with the detected tags highlighted.

    **Parameters:**  
    - `tag_family` (string, default: `tag36h11`): AprilTag family to use for detection.
    - `tag_border` (int, default: `1`): Width of the tag outer black border.
    - `tag_threads` (int, default: `4`): Number of threads the Apriltag 3 core algorithm should use.
    - `tag_decimate` (double, default: `1.0`): Decimation factor for quad detection.
    - `tag_blur` (double, default: `0.0`): Standard deviation for Gaussian blur applied to the segmented image.
    - `tag_refine_edges` (int, default: `1`): Whether to adjust quad edges to strong gradients nearby.
    - `tag_refine_decode` (int, default: `0`): Whether to refine detections to increase the number of detected tags.
    - `tag_refine_pose` (int, default: `0`): Whether to refine detections to increase pose accuracy.
    - `publish_tf` (bool, default: `false`): Enable publishing tag-camera relative poses on `/tf`.
    - `camera_frame` (string, default: `camera`): Camera frame name.
    - `publish_tag_detections_image` (bool, default: `false`): Enable publishing on `/tag_detections_image`.

    **Provided tf Transforms:**  
    - `tag` → `camera`: Relative pose of the camera frame to the tag frame.

2. **apriltag_ros_single_image_server**

    **Description:**  
    This node detects tags in a single provided image via a ROS2 service. It doesn't subscribe to topics or publish topics like the main `apriltag_ros` node.

    **Services:**  
    - `single_image_tag_detection` (`apriltag_ros/AnalyzeSingleImage`): Takes in the absolute file path of the input image, the absolute file path where to store the output image, and the camera intrinsics (particularly the K matrix). Returns detected tags' and tag bundles' poses.

3. **apriltag_ros_single_image_client**

    **Description:**  
    This node is a client for the `single_image_tag_detection` service provided by `apriltag_ros_single_image_server`.

    **Services Called:**  
    - `single_image_tag_detection` (`apriltag_ros/AnalyzeSingleImage`): See description above.

    **Arguments:**
    - `image_load_path` (string): Absolute file path of the image to detect tags in.
    - `image_save_path` (string): Absolute file path where to save the image with detected tags highlighted.

    **Parameters:**
    - `fx` (double): Camera x focal length (in pixels).
    - `fy` (double): Camera y focal length (in pixels).
    - `cx` (double): Camera image principal point x coordinate (in pixels).
    - `cy` (double): Camera image principal point y coordinate (in pixels).

## Tutorials
The main idea is to fill out `config/tags.yaml` with the standalone tags and tag bundles which you would like to detect (bundles potentially require a calibration process). Then, you simply run the continuous or single image detector

### Detection in a single image


The single image detector is based on a ROS2 service. Set the camera intrinsics fx, fy, cx and cy and provide a file path to the image. Then, the server runs it through the AprilTag detector and provides you the detection results and saves an output image.

Begin by running the service's server node:

```
ros2 launch apriltag_ros single_image_server_launch.py
```

Now you can run the client in order to detect tags and tag bundles in your image:
```
ros2 launch apriltag_ros single_image_client_launch.py image_load_path:=<FULL PATH TO INPUT IMAGE> image_save_path:=<FULL PATH TO OUTPUT IMAGE>
```

PNG images work well (others may work too, but have not been tested). The client will run and you will see the server print out Done! if everything is successful (the server will continue running, waiting for another single image detection service call). The output image will be at your indicated output image path.

### Detection in a video stream
The most common use case for this package is to detect tags in a continuous stream of images (video frames) coming from a camera. The camera name and image topic can be remapped to your implementation. Furthermore, you can set the `publish_tag_detections_image=false` in order to not publish the /tag_detections_image (more efficient if you do not care to see an image with the detected tags highlighted).

With the parameters correctly configured and the camera image stream being published, running the detector is trivial: 

```
ros2 launch apriltag_ros launch_continuous.py
```

## Contributing

Pull requests are welcome! Especially for the following areas:

- Publishing of the AprilTag 3 algorithm intermediate images over a ROS image topic (which AprilTag 3 already generates when `tag_debug==1`)
- Conversion of the bundle calibration script from MATLAB to Python
- Extend calibration to support calibrating tags that cannot appear simultaneously with the master tag, but do appear simultaneously with other tags which themselves or via a similar relationship appear with the master tag (e.g. a bundle with the geometry of a cube - if the master is on one face, tags on the opposite face cannot currently be calibrated). This is basically "transform chaining" and potentially allows calibration of bundles with arbitrary geometry as long as a transform chain exists from any tag to the master tag
- Supporting multiple tag family detection (currently all tags have to be of the same family). This means calling the detector once for each family. Because the core AprilTag 2 algorithm is the performance bottleneck, detection of `n` tag families will possibly decrease performance by `1/n` (tbd if this still holds for v3)

## Copyright

The source code in `apriltag_ros/` is original code that is the ROS wrapper itself, see the [LICENSE](https://github.com/AprilRobotics/apriltag_ros/blob/526b9455121ae0bb6b4c1c3db813f0fbdf78393c/LICENSE). It is inspired by [apriltags_ros](https://github.com/RIVeR-Lab/apriltags_ros) and provides a superset of its functionalities.

If you use this code, please kindly inform [Danylo Malyuta](mailto:danylo.malyuta@gmail.com) (to maintain a list here of research works that have benefited from the code) and cite:

- D. Malyuta, C. Brommer, D. Hentzen, T. Stastny, R. Siegwart, and R. Brockers, “[Long-duration fully autonomous operation of rotorcraft unmanned aerial systems for remote-sensing data acquisition](https://onlinelibrary.wiley.com/doi/abs/10.1002/rob.21898),” Journal of Field Robotics, p. arXiv:1908.06381, Aug. 2019.
- C. Brommer, D. Malyuta, D. Hentzen, and R. Brockers, “[Long-duration autonomy for small rotorcraft UAS including recharging](https://ieeexplore.ieee.org/document/8594111),” in IEEE/RSJ International Conference on Intelligent Robots and Systems, IEEE, p. arXiv:1810.05683, oct 2018.
- J. Wang and E. Olson, "[AprilTag 2: Efficient and robust fiducial detection](http://ieeexplore.ieee.org/document/7759617/)," in ''Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)'', October 2016.

```
@article{Malyuta2019,
  doi = {10.1002/rob.21898},
  url = {https://doi.org/10.1002/rob.21898},
  pages = {arXiv:1908.06381},
  year = {2019},
  month = aug,
  publisher = {Wiley},
  author = {Danylo Malyuta and Christian Brommer and Daniel Hentzen and Thomas Stastny and Roland Siegwart and Roland Brockers},
  title = {Long-duration fully autonomous operation of rotorcraft unmanned aerial systems for remote-sensing data acquisition},
  journal = {Journal of Field Robotics}
}
@inproceedings{Brommer2018,
  doi = {10.1109/iros.2018.8594111},
  url = {https://doi.org/10.1109/iros.2018.8594111},
  pages = {arXiv:1810.05683},
  year  = {2018},
  month = {oct},
  publisher = {{IEEE}},
  author = {Christian Brommer and Danylo Malyuta and Daniel Hentzen and Roland Brockers},
  title = {Long-Duration Autonomy for Small Rotorcraft {UAS} Including Recharging},
  booktitle = {{IEEE}/{RSJ} International Conference on Intelligent Robots and Systems}
}
@inproceedings{Wang2016,
  author = {Wang, John and Olson, Edwin},
  booktitle = {2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  doi = {10.1109/IROS.2016.7759617},
  isbn = {978-1-5090-3762-9},
  month = {oct},
  pages = {4193--4198},
  publisher = {IEEE},
  title = {{AprilTag 2: Efficient and robust fiducial detection}},
  year = {2016}
}
```
