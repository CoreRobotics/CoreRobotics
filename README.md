[![pipeline status](https://gitlab.com/powan/CoreRobotics/badges/master/pipeline.svg)](https://gitlab.com/powan/CoreRobotics/commits/master)
[![DOI](http://joss.theoj.org/papers/10.21105/joss.00489/status.svg)](https://doi.org/10.21105/joss.00489)
[![codecov][https://codecov.io/gh/CoreRobotics/CoreRobotics/branch/master/graph/badge.svg]][https://codecov.io/gh/CoreRobotics/CoreRobotics]

# CoreRobotics Project
#### Copyright (c) 2017
#### University of Washington
#### College of Engineering

- C++ platforms: Windows 8.1/10, Mac OS X 10.14, and Ubuntu.
- Python platforms: Mac, Ubuntu
- Matlab platforms: mac, ubuntu

## Description:
This git project hosts code for the CoreRobotics open source robotic control library.  The CoreRobotics libraries were developed in an effort to provide generalized implementations of algorithms facilitating rapid development of real-time robot manipulator control.  This is useful for roboticists and researchers who need full access to the control loop for high-performance applications, and can't afford the overhead associated with larger libraries such as the Robot Operating System (ROS).  CoreRobotics utilizes an object-oriented approach in C++ to implement fast cross-platform thread management and timing, core math solvers, manipulator control, and trajectory shaping, and modeling for state estimation.

An example application that makes use of the library runs a single-board computer (e.g.: a raspberry PI or similar computer) to control the motion of a small four-jointed robot arm.  A controller that solves for the joint angles to achieve a desired position of the robot tool uses the CoreRobotics InverseKinematics class.  To achieve smooth motions between robot tool waypoints, the controller uses the CoreRobotics TrajectoryGenerator class.  The Robot class presents a convenient way to represent the robot and update the robot kinematics quickly when new sensor data becomes available.

CoreRobotics is distrubted under the [BSD-3-Clause license](https://opensource.org/licenses/BSD-3-Clause).  This allows unlimited redistribution as long as the copyright notices and disclaimers of warranty are maintained.  The third clause indicates permission is required to the use of names of contributors for endorsement in any derived work.


## Structure:
- */bin* executable artifacts (examples and  tests).
- */doc* contains the doxygen settings for generating html documentation.  Please install [doxygen](http://www.stack.nl/~dimitri/doxygen/) and run the doxyfile to generate.  The main documentation page will be *doc/html/index.html*
- */examples* constains example use cases.  Binaries are compiled to /bin.
- */lib* library artifacts (created on compile).
- */matlab* swig bindings
- */python* swig bindings
- */scripts* helpful bash routines
- */src* library modules source and headers.  Binaries are compiled to /lib.
- */tests* unit tests (GTest).  Binaries are compiled to /bin.  Tests for each class are broken out into test_<classname> and should cover the class.
- *examples/* use of the library modules.  Binaries are compiled to bin.

## Dependencies
- [Required] Eigen (vector/matrix math header only) [http://eigen.tuxfamily.org/](http://eigen.tuxfamily.org/)
- [Required] Boost libraries (header only) [http://www.boost.org/](http://www.boost.org/)
- [Optional] Google test [https://github.com/google/googletest](https://github.com/google/googletest))
- [Optional] Open scene graph (OSG) [http://www.openscenegraph.org/](http://www.openscenegraph.org/)

On Mac with homebrew:
```
$ brew install eigen
$ brew install boost
$ brew install open-scene-graph
$ git clone https://github.com/google/googletest
$ cd googletest
$ mkdir build
$ cd build
$ cmake ..
$ make
$ make install
```

On Linux with apt-get:
```
$ apt-get install libeigen3-dev
$ apt-get install libboost-all-dev
$ apt-get install openscenegraph
$ sudo apt-get install libgtest-dev
$ cd /usr/src/gtest
$ mkdir build
$ cd build
$ cmake ..
$ make
$ cp *.a /usr/lib
```


## Compiling
CoreRobotics use CMake to manage build instructions.  To build CoreRobotics for development:
1. Install cmake [https://cmake.org/](https://cmake.org/). On mac,
```
$ brew install cmake
```
2. Set up the cmake build folder
```
$ mkdir <path_to_cr>/build
$ cd <path_to_cr>/build
```
3. Run cmake
```
$ cmake -D[option]=true -G "<compiler>" ..
```  
with options: `all` to set up all targets, `full` to build deprecated static library (all "CR" prefix classes),  `lib` to compile library, `tests` to build the tests, `examples` to build the examples, `python` to set python version/set wrapper target lib, `matlab` to set wrapper target lib.

Use  `cmake --help` for a list of available compilers.  Note that if you have not installed Eigen and Boost using one of the methods outlined in the External Dependencies section above (i.e. the packages aren't found by CMake), you must manually specify the path to boost and eigen headers using cmake flags, e.g.:  `cmake -G "<compiler>" -DEIGEN3_INCLUDE_DIR=<path to eigen3> -DBoost_INCLUDE_DIR=<path to boost> ../`  We provide a convenience repository for the needed 3rd party dependencies at [https://gitlab.com/powan/CRexternal](https://gitlab.com/powan/CRexternal)

4. Build.  On mac/Linux
```
$ make
$ make install
```
[Known issue on ubuntu docker image] You need to run the following command so the docker image can find the share libraries
```
$ ldconfig /usr/local/lib
```
5. Run the tests
```
$ cr-tests
```


## Use
To use in your own project, here's an example CMakeLists.txt for "target" and "main.cpp":
```
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD REQUIRED ON)
find_package(CoreRobotics REQUIRED)
include_directories(${CR_INCLUDE_DIRS})
link_directories(${CR_LIBRARY_DIRS})
add_executable("target" main.cpp)
target_link_libraries("target" ${CR_LIBRARIES})
```


## Support
We use the Service Desk feature in Gitlab to offer support.  If you are a user but not contributing to the library, please email your issues or questions to: [incoming+powan/CoreRobotics@incoming.gitlab.com](mailto:incoming+powan/CoreRobotics@incoming.gitlab.com).  You'll receive an email notification that the issue went through, and we'll get to work on addressing it!  If you are a contributor, please submit your issues through the issue tracker feature in Gitlab.
