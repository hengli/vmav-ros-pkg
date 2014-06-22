vmav-ros-pkg
============

ROS packages for vision-based MAVs.

Required dependencies:

1. [px-ros-pkg](https://github.com/cvg/px-ros-pkg)
2. [ethzasl_sensor_fusion (branch: catkin)](https://github.com/cvg/ethzasl_sensor_fusion)
3. [asctec_mav_framework (branch: experimental)](https://github.com/cvg/asctec_mav_framework)
4. Boost >= 1.4.0 (Ubuntu package: libboost-all-dev)
5. Eigen3 (Ubuntu package: libeigen3-dev)
6. gflags (Ubuntu package: libgflags-dev)
7. glog ([Source install](https://github.com/schuhschuh/gflags/archive/v2.1.1.tar.gz))
8. OpenCV >= 2.4.8
9. SuiteSparse >= 4.2.1 ([Source install](https://www.cise.ufl.edu/research/sparse/SuiteSparse/SuiteSparse-4.2.1.tar.gz))
10. RTI Connext DDS >= 5.1.0 ([Source install](http://www.rti.com/downloads/connext-files.html))

If you use the packages for an academic publication, please cite the following paper:

    Lionel Heng, Gim Hee Lee, and Marc Pollefeys,
    Self-Calibration and Visual SLAM with a Multi-Camera System on a Micro Aerial Vehicle,
    In Proc. Robotics: Science and Systems (RSS), 2014.

#### Acknowledgements ####

The primary author, Lionel Heng, is funded by the DSO Postgraduate Scholarship. This work is partially supported by the SNSF V-MAV grant (DACH framework).

The repository includes third-party code from the following sources:

    1. M. Rufli, D. Scaramuzza, and R. Siegwart,
       Automatic Detection of Checkerboards on Blurred and Distorted Images,
       In Proc. IEEE/RSJ International Conference on Intelligent Robots and Systems, 2008.

    2. Sameer Agarwal, Keir Mierle, and Others,
       Ceres Solver.
       https://code.google.com/p/ceres-solver/
        
    3. D. Galvez-Lopez, and J. Tardos,
       Bags of Binary Words for Fast Place Recognition in Image Sequences,
       IEEE Transactions on Robotics, 28(5):1188-1197, October 2012.

    4. L. Kneip, D. Scaramuzza, and R. Siegwart,
       A Novel Parametrization of the Perspective-Three-Point Problem for a
       Direct Computation of Absolute Camera Position and Orientation,
       In Proc. IEEE Conference on Computer Vision and Pattern Recognition, 2011.

    5. pugixml
       http://pugixml.org/

    6. E. Olson,
       AprilTag: A robust and flexible visual fiducial system,
       In Proc. IEEE International Conference on Robotics and Automation, 2011.
