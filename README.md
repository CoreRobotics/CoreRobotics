[![pipeline status](https://gitlab.com/powan/CoreRobotics/badges/master/pipeline.svg)](https://gitlab.com/powan/CoreRobotics/commits/master)
[![DOI](http://joss.theoj.org/papers/10.21105/joss.00489/status.svg)](https://doi.org/10.21105/joss.00489)

# CoreRobotics Project
#### Copyright (c) 2017
#### University of Washington
#### College of Engineering

Created: 2016   
Version: 1.1.0

- C++ Platforms: Windows 8.1/10, Mac OS X 10, and Linux.

## Description:
This git project hosts code for the CoreRobotics open source robotic control library.  The CoreRobotics libraries were developed in an effort to provide generalized implementations of algorithms facilitating rapid development of real-time robot manipulator control.  This is useful for roboticists and researchers who need full access to the control loop for high-performance applications, and can't afford the overhead associated with larger libraries such as the Robot Operating System (ROS).  CoreRobotics utilizes an object-oriented approach in C++ to implement fast cross-platform thread management and timing, core math solvers, manipulator control, and trajectory shaping, and modeling for state estimation.

An example application that makes use of the library runs a single-board computer (e.g.: a raspberry PI or similar computer) to control the motion of a small four-jointed robot arm.  A controller that solves for the joint angles to achieve a desired position of the robot tool uses the CoreRobotics InverseKinematics class.  To achieve smooth motions between robot tool waypoints, the controller uses the CoreRobotics TrajectoryGenerator class.  The Manipulator class presents a convenient way to represent the robot and update the robot kinematics quickly when new sensor data becomes available.

The CoreRobotics library is distrubted under the [BSD-3-Clause license](https://opensource.org/licenses/BSD-3-Clause).  This allows unlimited redistribution as long as the copyright notices and disclaimers of warranty are maintained.  The third clause indicates permission is required to the use of names of contributors for endorsement in any derived work.

The best way to get involved is to read this document, check out the library, and start contributing code by submitting a merge request!


## General Structure:
- *doc/* contains the doxygen Doxyfile for generating html documentation.  Please install [doxygen](http://www.stack.nl/~dimitri/doxygen/) and run the doxyfile to generate.  The main documentation page will be *doc/html/index.html*
- *python/* contains code for the python wrapper.
- *src/* contains the library source and headers.  "CoreRobotics.hpp" is the main header for the project.  Classes and functions are organized into sub folders.
- *tests/* contains code for testing library functionality (Using GTest).  Binaries are compiled to tests/bin.  Tests for each class are broken out into test_<classname> and should test functionality of each method.  The test binary gets written to *./bin*  These scripts are also useful for seeing examples of how to use a particular function of the library.
- *examples/* constains examples.  Binaries are compiled to examples/bin.

## External Dependencies
CoreRobotics makes use of Eigen (vector/matrix math) and Boost libraries.  Make sure you have installed these for your system before proceeding.  Only headers from both libraries are needed, so it is not necessary to compile binaries.  If you are using Mac, you can use homebrew to install.  On Linux, you can use apt-get.
1. Eigen can be found at [http://eigen.tuxfamily.org/](http://eigen.tuxfamily.org/).  Using homebrew (Mac), the install is
`brew install eigen`
On Linux Eigen can be installed by download an archive from
[http://eigen.tuxfamily.org/](http://eigen.tuxfamily.org/)
and following the directions in the contained `INSTALL` file.
2. Boost can be found at [http://www.boost.org/](http://www.boost.org/).  Using homebrew (Mac), the install is
`brew install boost`
On Linux using aptitude, the install is
`sudo apt-get install libboost-all-dev`
3. [Optional] GTest is used for the test suite, found at [https://github.com/google/googletest](https://github.com/google/googletest).  On Linux using aptitude, the install is
`sudo apt-get install libgtest-dev`
then the library needs to be compiled and installed by running
`cd /usr/src/gtest && sudo mkdir build && cd build && sudo cmake ../ && sudo make && sudo cp *.a /usr/lib`


## Building and compiling
CoreRobotics relies on CMake to compile cross-platform source to platform-specific libraries and binaries.  To build and compile CoreRobotics for development, follow these steps:
1. Download CMake for your platform at [https://cmake.org/](https://cmake.org/).  Make sure you have installed for command line use. (See CMake UI: Tools>How to Install for Command Line Use)
2. Open terminal (or emulator) and change directory (`cd <CoreRoboticsRoot>/`) (replace <CoreRoboticsRoot> for your specific setup with the root directory of the project.)
3. Create a folder `mkdir build`.
4. Change directories `cd build`.
5. Run CMake `cmake -G "<compiler>" ../`  To get a list of compilers available to configure with CMake, type `cmake --help`.  CMake will check for the dependencies.  Note that if you have not installed Eigen and Boost using one of the methods outlined in the External Dependencies section above (i.e. the packages aren't found by CMake), you must manually specify the path to boost and eigen headers using cmake flags, e.g.:  `cmake -G "<compiler>" -DEIGEN3_INCLUDE_DIR=<path to eigen3> -DBoost_INCLUDE_DIR=<path to boost> ../`  We provide a convenience repository for the needed 3rd party dependencies at [https://gitlab.com/powan/CRexternal](https://gitlab.com/powan/CRexternal) if you don't want to manage these packages.
6. Open the project in the *build\* directory and build accordingly. (e.g.: For visual studio, a CoreRobotics.sln will be created.  Open this solution and build all.  The library and binaries will be compiled by the Visual Studio IDE.)
7. To test that the library works, there are 2 options:  If GTest is installed on your machine, the tests will be build to `tests/bin` - all green "OK's" means everything is working properly.  The examples will be compiled to `examples/bin`.  Running `example_core` checks basic multithreading capability and should output the following:
`**********************
Demonstration of CRCore
t = 0.101898
Thread 1: i = 1
Thread 2: i = 1
Thread 1: i = 2
Thread 1: i = 3
Thread 2: i = 2
Thread 1: i = 4
Thread 1: i = 5
Thread 2: i = 3
Thread 1: i = 6
Thread 1: i = 7
Thread 1: i = 8
Thread 2: i = 4
Thread 1: i = 9
Thread 1: i = 10`



## Using the library
When built using your CMake-generated project, the CoreRobotics creates folders examples/bin and examples/test (if GTest is installed) which contain the binary example and test scripts.  Make sure these execute on build.  The following is an example project CMakeLists.txt for "MyExecutable" and a simple main.cpp:
`SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD REQUIRED ON)
find_package(CoreRobotics REQUIRED)
include_directories(${CR_INCLUDE_DIRS})
link_directories(${CR_LIBRARY_DIRS})
add_executable("MyExecutable" main.cpp)
target_link_libraries("MyExecutable" ${CR_LIBRARIES})
`


## Support:
We use the Service Desk feature in Gitlab to offer support.  If you are a user but not contributing to the library, please email your issues or questions to: [incoming+powan/CoreRobotics@incoming.gitlab.com](mailto:incoming+powan/CoreRobotics@incoming.gitlab.com).  You'll receive an email notification that the issue went through, and we'll get to work on addressing it!  If you are a contributor, please submit your issues through the issue tracker feature in Gitlab.


## Developer Guidelines:
For general questions about git, see the [Gitlab help pages](https://gitlab.com/help/)

1. Absolutely all class methods must have existing references in peer-reviewed publications (text books, conference papers, journal papers, etc…), and the reference must be included in the source code.  Do not add your fancy algorithms here until you’ve published.  If you’re considering putting something into the library that has a lot of data, it doesn’t belong there - this is a project for implementing generic robotics math, not specific applications.
2. The project makes extensive use of object-oriented programming (class definitions). If you are unsure about what this is, refer to the [C++ Reference](http://www.learncpp.com/cpp-tutorial/81-welcome-to-object-oriented-programming/).
3. Keep the C++ code crossplatform (i.e. don’t use OS-specific functions unless you check the OS and have a method for each OS.  Plan to support Windows 8.1/10, Mac OSX, and Linux Ubuntu.
4. Any general math functions that you think might come in handy in someone else’s work should be included in the Math class.  This is a static method class that acts as a container to hold all the useful math routines.
5. If you use a 3rd party library for your contribution, make sure its license (specifically derived-works) is compatible with the BSD 3-clause license. Include the library source in the *external/* folder and make sure the documentation for that implementation acknowledges the use of the library.
6. Comment your classes for Doxygen.  Following existing code for examples on how to do this.
7. Use the *utils/* folder to write tests for your contirbutions.
8. Please keep the code in working condition.  Only commit changes you've made to code if it is working properly (this means compile and run BEFORE you commit!).


## Brief Style Guide
Refer to the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) for detailed explanations.
- Use intuitive naming. Use discretion with acronymns or when abbreviating anything that isn’t well known.
- Classes should have `CR` in front of the name, e.g.: `CRTransferFunction`
- Methods and members should use Camel case, e.g.: `setProperty(<args>)`, `getTransferFunction()`
- Use upper case and underscores for enumerators, e.g.: `MY_CLEVER_ENUMERATOR`.
- Class members should be protected, and setting/getting the member value should be defined explicitly.  The exception is when a class member is a type associated with a 3rd party library (this is so one can access the property without having to modify the containing CoreRobotics class)
- All class members should have the `m_` prefix for readibility, e.g.: `m_classProperty`
- Method arguments should utilize the `i_` prefix to indicate an input argument, e.g.: `i_transferFunction`, and the `o_` prefix to indicate an output argument, e.g.: `o_transferFunction`
- Consider passing input arguments by value or pointer and output arguments by reference, e.g.: `CRClass::solveProblem(double i_argInOne, double* i_argInTwo, double& o_argOut)`.  Use discretion when making exceptions.
- When a method returns only one output, use the return, e.g.: `double getProperty(void)`.  If the method has more than one output, pass by function argument, e.g.: `void getProperties(double& o_propOne, double& o_propTwo)`, and reserve the return for indicating a result.
- If a result is returned, use the `CoreRobotics::CRResult` enumerator (contained in `CoreRobotics::CRTypes`).  Add result types as needed and comment which results can be expected by a particular method.


## Git commands for reference:
Some basic git commands through the command line are:
- `git status` check the status of repository.
- `git add "<path/file>"` add the file to the git path.
- `git add .` add all the files you changed in your local directory.
- `git commit -m "<commit message>"` commit the change.  Only do this if your code is working!
- `git push <branch>` pushes the commits to the specified branch.
- `git branch` lists the branches
- `git checkout <branch>` checks out the specified branch
- `git pull` pulls new commits from the server.


Happy developing!


-Parker
