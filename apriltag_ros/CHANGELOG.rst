^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package apriltag_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.1 (2022-04-05)
------------------
* Fixed propagation of apriltag and Eigen headers and libraries (`#124 <https://github.com/AprilRobotics/apriltag_ros/issues/124>`_)
* Drop old C++11 as it breaks with new log4cxx.
* Contributors: Jochen Sprickerhof, Remo Diethelm, Wolfgang Merkt

3.2.0 (2022-03-10)
------------------
* Add transport hint option (`#108 <https://github.com/AprilRobotics/apriltag_ros/issues/108>`_)
* Move to using the apriltag CMake target (`#104 <https://github.com/AprilRobotics/apriltag_ros/issues/104>`_)
* Set the tag's parent frame to the camera optical frame (`#101 <https://github.com/AprilRobotics/apriltag_ros/issues/101>`_)
* Fix bug in K matrix in single_image_client (`#103 <https://github.com/AprilRobotics/apriltag_ros/issues/103>`_)
* Add configurable max_hamming_distance for the AprilTag Detector (`#93 <https://github.com/AprilRobotics/apriltag_ros/issues/93>`_)
* Introduce lazy processing for ContinuousDetector (`#80 <https://github.com/AprilRobotics/apriltag_ros/issues/80>`_)
* Contributors: Akshay Prasad, Amal Nanavati, Christian Rauch, Hongzhuo Liang, Wolfgang Merkt

3.1.2 (2020-07-15)
------------------
* Add support for tagCircle21h7, tagCircle49h12 (`#69 <https://github.com/AprilRobotics/apriltag_ros/issues/69>`_)
* Add support for tagCustom48h12 (`#65 <https://github.com/AprilRobotics/apriltag_ros/issues/65>`_)
* Contributors: Anthony Biviano, Kyle Saltmarsh, Wolfgang Merkt, kai wu

3.1.1 (2019-10-07)
------------------
* Add support for tagStandard41h12 and tagStandard52h13 (`#63 <https://github.com/AprilRobotics/apriltag_ros/issues/63>`_, `#59 <https://github.com/AprilRobotics/apriltag_ros/issues/59>`_).
* Add gray image input support (`#58 <https://github.com/AprilRobotics/apriltag_ros/issues/58>`_).
* Contributors: Alexander Reimann, Moritz Zimmermann, Samuel Bachmann, Wolfgang Merkt

3.1.0 (2019-05-25)
------------------
* Prepare for release (3.1) and fix catkin_lint errors
* Upgrade to AprilTag 3, fix installation, and performance improvements (`#43 <https://github.com/AprilRobotics/apriltag_ros/issues/43>`_)
  Upgrade to AprilTag 3, fix installation, and performance improvements
* Rename package to apriltag_ros
  - Relates to https://github.com/AprilRobotics/apriltag_ros/issues/45
* Contributors: Wolfgang Merkt

1.0.0 (2018-05-14)
------------------
