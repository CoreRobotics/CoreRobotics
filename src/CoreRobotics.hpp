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
including kinematics, estimation, perception, modeling and representations of
items in the world. The libraries are written in C++ and are intended to be
cross-platform, although they have currently only been tested on Windows 
8.1/10, Mac OS X, and Ubuntu platforms.  The code is distributed under the
BSD 3-clause license for flexibility of use.  By using this software, you are
agreeing to the license.  Please read the license prior to installing the
CoreRobotics library.
 
This introduction is broken down into the following sections.
- \subpage license
- \subpage install
- \subpage overview
- \subpage gettingstarted
- \subpage references
- \subpage python
- \subpage matlab

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
- \subpage install_mac
- \subpage install_windows
\n\n

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
$ cmake -DCMAKE_BUILD_TYPE=Release -G "Xcode" ../
$ open CoreRobotics.xcodeproj/
\endcode
 
### Step 3.
After the project files are created, the CoreRobotics XCode project
should open.  Go to Product >> Build (Command - B) to build the project.
 
### Step 4.
Once the project has built, you can run the test script by running the
following terminal commands (assuming you left terminal alone from step 2)

\code
$ ../utils/bin/Debug/TestModules
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
utils, etc.  Run the following commands one at a time to build the CoreRobotics
library, test routines, and example code:

\code
> mkdir build
> cd build
> cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 14 2015" ../
> CoreRobotics.sln
\endcode

### Step 3.
After the project files are created, the CoreRobotics Visual Studio project
should open.  Go to Build >> Build Solution (F7) to build the project.

### Step 4.
Once the project has built, you can run the test script by running the
following terminal commands (assuming you left terminal alone from step 2)

\code
> ..\utils\bin\Release\TestModules.exe
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
 and rigid body representations for kinematics and dynamics
- \subpage models implements models such as dynamics, sensor, and noise
 models, as well as a class for handling manupulators.
- \subpage controllers implements controllers and planners
- \subpage estimators implements state estimtion and perceptions
- \subpage world implements scene graph world and child representations.
 
 */
//---------------------------------------------------------------------------
/*!
\page gettingstarted Getting Started
This section assumes you have properly installed CoreRobotics for your 
platform.  It is also recommended that you've read through the Overview
section prior to tackling some of the examples presented here.

\section transformations Transformations
\section kinematics Kinematics
\section inversekinematics Inverse Kinematics
\section motion Motion Modeling
\section estimation State Estimation
Todo
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
\brief CoreRobotics header.
//! \details Todo
*/
//===========================================================================


//---------------------------------------------------------------------------
//! \defgroup core Core
//! \brief Implements core functionality.
//! \details This module implements the core functionality of the library,
//! such as timing, loop definitions and threading.
//---------------------------------------------------------------------------
#include "CRTypes.hpp"
#include "CRClock.hpp"
#include "CRThread.hpp"


//---------------------------------------------------------------------------
//! \defgroup math Math
//! \brief Implements math components.
//! \details Todo
//---------------------------------------------------------------------------
#include "CRMath.hpp"


//---------------------------------------------------------------------------
//! \defgroup physics Physics
//! \brief Implements physics (kinematics and dynamics) representations.
//! \details Todo
//---------------------------------------------------------------------------
#include "CRFrame.hpp"
#include "CRFrameEuler.hpp"
#include "CRFrameDh.hpp"
#include "CRRigidBody.hpp"


//---------------------------------------------------------------------------
//! \defgroup models Models
//! \brief Implements model (sensor, motion, and noise) representations.
//! \details Todo
//---------------------------------------------------------------------------
#include "CRManipulator.hpp"
#include "CRNoiseModel.hpp"
#include "CRNoiseGaussian.hpp"
#include "CRNoiseDirac.hpp"
#include "CRNoiseUniform.hpp"
#include "CRNoiseMixture.hpp"
#include "CRSensorModel.hpp"
#include "CRSensorLinear.hpp"
#include "CRSensorProbabilistic.hpp"
#include "CRMotionModel.hpp"
#include "CRMotionLinear.hpp"
#include "CRMotionProbabilistic.hpp"


//---------------------------------------------------------------------------
//! \defgroup world World
//! \brief Implements the world for interacting with multiple objects.
//! \details Todo
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
//! \defgroup estimators Estimators
//! \brief Implements estimators for recovering model states.
//! \details Todo
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
//! \defgroup controllers Controllers
//! \brief Implements controllers and policies for regulating motion and actions.
//! \details Todo
//---------------------------------------------------------------------------
#include "CRInverseKinematics.hpp"
#include "CRNullSpace.hpp"
#include "CRHardLimits.hpp"
#include "CRTrajectoryGenerator.hpp"


#endif
