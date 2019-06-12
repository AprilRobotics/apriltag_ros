# apriltag_ros

`apriltag_ros` is a Robot Operating System (ROS) wrapper of the [AprilTag 3 visual fiducial detector](https://april.eecs.umich.edu/software/apriltag.html). For details and tutorials, please see the [ROS wiki](http://wiki.ros.org/apriltag_ros).

`apriltag_ros` depends on the latest release of the [AprilTag library](https://github.com/AprilRobotics/apriltag). Clone it into your catkin workspace before building.

**Authors**: Danylo Malyuta, Wolfgang Merkt

**Maintainers**: [Danylo Malyuta](mailto:danylo.malyuta@gmail.com) (NASA Jet Propulsion Laboratory, California Institute of Technology), [Wolfgang Merkt](https://github.com/wxmerkt)

## Quickstart

Starting with a working ROS installation (Kinetic and Melodic are supported):
```
export ROS_DISTRO=melodic               # Set this to your distro, e.g. kinetic or melodic
source /opt/ros/$ROS_DISTRO/setup.bash  # Source your ROS distro 
mkdir -p ~/catkin_ws/src                # Make a new workspace 
cd ~/catkin_ws/src                      # Navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper
cd ~/catkin_ws                          # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
catkin build    # Build all packages in the workspace (catkin_make_isolated will work also)
```
See the [ROS wiki](http://wiki.ros.org/apriltag_ros) for details and tutorials.

## Contributing

Pull requests are welcome! Especially for the following areas:

- Publishing of the AprilTag 3 algorithm intermediate images over a ROS image topic (which AprilTag 3 already generates when `tag_debug==1`)
- Conversion of the bundle calibration script from MATLAB to Python
- Extend calibration to support calibrating tags that cannot appear simultaneously with the master tag, but do appear simultaneously with other tags which themselves or via a similar relationship appear with the master tag (e.g. a bundle with the geometry of a cube - if the master is on one face, tags on the opposite face cannot currently be calibrated). This is basically "transform chaining" and potentially allows calibration of bundles with arbitrary geometry as long as a transform chain exists from any tag to the master tag
- Supporting multiple tag family detection (currently all tags have to be of the same family). This means calling the detector once for each family. Because the core AprilTag 2 algorithm is the performance bottleneck, detection of `n` tag families will possibly decrease performance by `1/n` (tbd if this still holds for v3)

## Changelog

- In March 2019, the code was upgraded to AprilTag 3 and as thus the options `refine_pose`, `refine_decode`, and `black_border` were removed.

## Copyright

The source code in `apriltag_ros/` is original code that is the ROS wrapper itself, see the [LICENSE](https://github.com/AprilRobotics/apriltag_ros/blob/526b9455121ae0bb6b4c1c3db813f0fbdf78393c/LICENSE). It is inspired by [apriltags_ros](https://github.com/RIVeR-Lab/apriltags_ros) and provides a superset of its functionalities.

If you use this code, please kindly cite:

- D. Malyuta, “[Navigation, Control and Mission Logic for Quadrotor Full-cycle Autonomy](https://www.research-collection.ethz.ch/handle/20.500.11850/248154),” Master thesis, Jet Propulsion Laboratory, 4800 Oak Grove Drive, Pasadena, CA 91109, USA, December 2017.
- J. Wang and E. Olson, "[AprilTag 2: Efficient and robust fiducial detection](http://ieeexplore.ieee.org/document/7759617/)," in ''Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)'', October 2016.

```
@mastersthesis{malyuta:2017mt,
  author = {Danylo Malyuta},
  title = {{Guidance, Navigation, Control and Mission Logic for Quadrotor Full-cycle Autonomy}},
  language = {english},
  type = {Master thesis},
  school = {Jet Propulsion Laboratory},
  address = {4800 Oak Grove Drive, Pasadena, CA 91109, USA},
  month = dec,
  year = {2017}
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
