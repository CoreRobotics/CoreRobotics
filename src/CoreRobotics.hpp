//===========================================================================
/*
Software License Agreement (BSD-3-Clause License)
Copyright (c) 2017, CoreRobotics.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

* Neither the name of CoreRobotics nor the names of its contributors
may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

\project CoreRobotics Project
\url     www.corerobotics.org
\author  Parker Owan, Cameron Devine, Tony Piaskowy

*/
//===========================================================================

#ifndef corerobotics_hpp
#define corerobotics_hpp

//===========================================================================
/*! \mainpage Introduction

The CoreRobotics libraries provide basic modules for real-time robot control
including kinematics, inverse kinematics, trajectory shaping, control,
estimation, modeling, multi-threading and timing. The libraries are written
in C++ and are intended to be cross-platform, currently tested on Windows
8.1/10, Mac OS X, and Ubuntu platforms.  The code is distributed under the
BSD 3-clause license for flexibility of use.  By using this software, you are
agreeing to the license.  Please read the license prior to using the
CoreRobotics library.
 
This introduction is broken down into the following sections.
- \subpage license
- \subpage install
- \subpage overview
- \subpage gettingstarted
- \subpage examplecmake
- \subpage python
- \subpage matlab
- \subpage references

A note on units:  All units in the library, unless specified, are in SI
(international standard), i.e.: radians, meters, kilograms, etc...
 
 
*/
//---------------------------------------------------------------------------
/*!
\page license Software License

Software License Agreement (BSD-3-Clause License)\n
Copyright (c) 2017, CoreRobotics.\n
All rights reserved.\n\n

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:\n

1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

3. Neither the name of CoreRobotics nor the names of its contributors
may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
 
*/
//---------------------------------------------------------------------------
/*!
\page install Installation

The CoreRobotics Library comes with CMakeLists ready to generate the project
for your IDE of choice.  CMake is a tool for generating platform-specific 
IDE files from cross-platform source code.  We highly recommend the use of 
CMake to build your first program.  Instructions for how to build CoreRobotics
for your specific platform using CMake are presented below.
- \subpage install_dependencies
- \subpage install_linux
- \subpage install_mac
- \subpage install_windows
\n\n
 
\section install_dependencies Install Dependencies
CoreRobotics requires Eigen3 and Boost libraries to be installed.  There are
several options to accomnplish this.  On Mac, you can use Homebrew to install
the pacakges https://brew.sh/  The following lines of code install homebrew and
the required dependencies
\code
$ /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
$ brew install eigen
$ brew install boost
\endcode
Note that if the first line fails, go to https://brew.sh/ to get the latest install link.
 
If you are using Linux, you can use aptitude to install boost using the following
command
\code
$ sudo apt-get install libboost-all-dev
\endcode
Eigen can be installed by going to https://eigen.tuxfamily.org and downloading an
archive of the repository. Then follow the directions in the contained `INSTALL` file.

If desired GTest can be installed by running
\code
$ sudo apt-get install libgtest-dev
$ cd /usr/src/gtest && sudo mkdir build && cd build && sudo cmake ../ && sudo make && sudo cp *.a /usr/lib
\endcode

If you are using Windows, you can download the necessary dependencies from our repository at
 https://gitlab.com/powan/CRexternal

 
 
\section install_linux Linux Make
Tested on Ubuntu 16.04.

### Step 1.
Download the latest version of CMake for Linux.
\code
$ sudo apt-get install cmake
\endcode

### Step 2.
After CMake is installed, use the terminal to cd into the root folder of
CoreRobotics.  There should be subfolders named doc, src, utils, etc.
Run the following commands one at a time to build the CoreRobotics
library, test routines, and example code:

\code
$ mkdir build
$ cd build
$ cmake -G "Unix Makefiles" ../
\endcode

### Step 3.
After the project files are created, you can build the library with:
\code
$ make
\endcode

### Step 4.
Once the project has built, you can run the test script by running the
following terminal commands (assuming you left terminal alone from step 2)

\code
$ cd ../bin/Release
$ ./TestModules
\endcode
\n\n\n\n

\section install_mac Mac XCode
Tested on Mac OSX 10.11.6 with XCode Version 8.1.  The remaining steps assume
you have already installed XCode on your Mac computer.
 
### Step 1.
Download the latest version of CMake from https://cmake.org/download/
for Mac OSX (*.dmg).  Install CMake by running the *.dmg.  Once CMake has
installed, it is accesible via GUI.  For instructions on how to set up CMake
for command line, open the CMake application, go to Tools > How to install for
command line use, and run one of the methods in the command line.
 
### Step 2.
Once CMake is set up for command line use, open terminal and cd into
the root folder of CoreRobotics.  There should be subfolders named doc, src, 
utils, etc.  Run the following commands one at a time to build the CoreRobotics
library, test routines, and example code:
 
\code 
$ mkdir build
$ cd build
$ cmake "Xcode" ../
$ open CoreRobotics.xcodeproj/
\endcode
 
### Step 3.
After the project files are created, the CoreRobotics XCode project
should open.  To change the build type, under "Edit Scheme", select the "Build
Configuration".  Then, go to Product >> Build (Command - B) to build the project.
 
### Step 4.
Once the project has built, you can run the test script by running the
following terminal commands (assuming you left terminal alone from step 2)

\code
$ cd ../bin/Release
$ ./TestModules
\endcode
\n\n\n\n
 
\section install_windows Windows Visual Studio
Tested on Windows 8.1 with Microsoft Visual Studio 2015.  The remaining steps
assume you have already installed Visual Studio 2015 on your PC.

### Step 1.
Download the latest version of CMake from https://cmake.org/download/
for Windows (*.msi).  Install CMake by running the *.msi.  Once CMake has
installed, it is accesible via GUI or the command line.

### Step 2.
Once CMake is set up for command line use, open terminal and cd into
the root folder of CoreRobotics.  There should be subfolders named doc, src,
utils, etc.  You will need to specify the path to the dependencies when running
CMake. Run the following commands one at a time to build the CoreRobotics
library, test routines, and example code:

\code
> mkdir build
> cd build
> cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 14 2015" DEIGEN3_INCLUDE_DIR=<path to eigen3> -DBoost_INCLUDE_DIR=<path to boost> ../
> CoreRobotics.sln
\endcode

### Step 3.
After the project files are created, the CoreRobotics Visual Studio project
should open.  Go to Build >> Build Solution (F7) to build the project.

### Step 4.
Once the project has built, you can run the test script by running the
following terminal commands (assuming you left terminal alone from step 2)

\code
>> cd ..\bin\Release
>> TestModules.exe
\endcode
\n\n\n\n
*/
//---------------------------------------------------------------------------
/*!
\page overview Overview
CoreRobotics is seperated into several modules, based on the natural
classifications of robotics literature and implementation.  The intention is
for each module to solve different problems in robotics applications.  Here
we present a light overview of the modules and briefly describe the use for
each class.
 
- \subpage math implements static methods for performing mathematical
 operations.
- \subpage core implements core classes (such as timing and multi-
 threading) for implementation.
- \subpage physics implements physics models such as frame transformations
 and rigid body representations for kinematics and dynamics.
- \subpage models implements models such as dynamics, sensor, and noise
 models, as well as a class for handling manupulators.
- \subpage controllers implements controllers and planners.
 
 */
//---------------------------------------------------------------------------
/*!
\page gettingstarted Getting Started
Once you have succesfully built CoreRobotics, it's time to check out some of
the examples.  Each class in the library has an associated script in the utils
folder that tests the functionality of the class.  Examining these examples is
the best way to get started with CoreRobotics.  The examples are conveniently
included in the class descriptions.

*/
//---------------------------------------------------------------------------
/*!
 \page examplecmake CMake Example
 To include CoreRobotics in your own project, we rccomend use of CMake.
 Add the following commands to your CMakeLists.txt  Be sure to replace items
 in brackets with specific names for your setup.
 
 \code
 set (CR_DIR <path/to/corerobotics/root>)
 
 include_directories(
     ${CR_DIR}/src
     ${CR_DIR}/src/core
     ${CR_DIR}/src/math
     ${CR_DIR}/src/models
     ${CR_DIR}/src/estimators
     ${CR_DIR}/src/physics
     ${CR_DIR}/src/controllers
     <path to eigen>
     <path to boost>
 )
 
 link_directories(${CR_DIR}/lib/Release)
 
 target_link_libraries(<target_name> ${CR_DIR}/lib/Release/<lib name>)
 \endcode
 */
//---------------------------------------------------------------------------
/*!
\page references References

 [1] J. Craig, "Introduction to Robotics: Mechanics and Control", Ed. 3, 
     Pearson, 2004. \n\n
 
 [2] J. Denavit and R. Hartenberg, "A kinematic notation for lower-pair 
     mechanisms based on matrices". Trans ASME J. Appl. Mech. 23, pp. 215-221, 1955. \n\n
 
 [3] R. Murray, Z. Li, and S. Sastry, "A Mathematical Introduction to Robot 
     Manipulation", Ed. 1, CRC Press, 1993. 
     <a href="http://www.cds.caltech.edu/~murray/mlswiki/index.php/Main_Page">
     Available online.</a> \n\n
 
 [4] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems", 
     Ed. 2, CRC Press, 2012. \n\n
 
 [5] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press, 2006. \n\n
 
*/
//===========================================================================

//===========================================================================
/*!
\file CoreRobotics.hpp
\brief This is the CoreRobotics header file.  Including this file in your
project includes all the CoreRobotics modules.
*/
//===========================================================================

#define _USE_MATH_DEFINES
#include <cmath>

//---------------------------------------------------------------------------
//! \defgroup core Core
//! \brief This module implements the core functionality of the library,
//! such as timing, loop definitions and threading.
//---------------------------------------------------------------------------
#include "CRTypes.hpp"
#include "CRClock.hpp"
#include "CRThread.hpp"
#include "CRLoop.hpp"
#include "CRLoopElement.hpp"
#include "CRMutex.hpp"
#include "CRSharedMemory.hpp"
#include "CRItem.hpp"


//---------------------------------------------------------------------------
//! \defgroup world World
//! \brief TEMPORARY DURING MAJOR REVISION TO 2.0
//---------------------------------------------------------------------------
#include "Signal.hpp"
#include "World.hpp"
#include "WorldItem.hpp"
#include "Model.hpp"
#include "Sensor.hpp"
#include "Actuator.hpp"
// #include "CRPolicy.hpp"


//---------------------------------------------------------------------------
//! \defgroup math Math
//! \brief Implements math components.
//---------------------------------------------------------------------------
#include "CRConversion.hpp"
#include "CRIntegration.hpp"
#include "CRMatrix.hpp"


//---------------------------------------------------------------------------
//! \defgroup physics Physics
//! \brief Implements physics (kinematics and dynamics) representations.
//---------------------------------------------------------------------------
#include "CRFrame.hpp"
#include "CRFrameEuler.hpp"
#include "CRFrameDh.hpp"
#include "CRRigidBody.hpp"


//---------------------------------------------------------------------------
//! \defgroup models Models
//! \brief Implements model (sensor, motion, and noise) representations.
//---------------------------------------------------------------------------
#include "CRManipulator.hpp"
#include "CRNoiseModel.hpp"
#include "CRNoiseGaussian.hpp"
#include "CRNoiseUniform.hpp"
#include "CRNoiseMixture.hpp"
#include "CRGmm.hpp"
#include "CRSensorModel.hpp"
#include "CRSensorLinear.hpp"
#include "CRSensorProbabilistic.hpp"
#include "CRMotionModel.hpp"
#include "CRMotionLinear.hpp"
#include "CRMotionProbabilistic.hpp"


//---------------------------------------------------------------------------
//! \defgroup controllers Controllers
//! \brief Implements controllers and policies for regulating motion and actions.
//---------------------------------------------------------------------------
#include "CRInverseKinematics.hpp"
#include "CRNullSpace.hpp"
#include "CRHardLimits.hpp"
#include "CRTrajectoryGenerator.hpp"


#endif
