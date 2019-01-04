//=====================================================================
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
 \author  Cameron Devine
 
 */
//=====================================================================


/*!
\page matlab Matlab Wrapper

A Matlab wrapper for a portion of the CoreRobotics library has been
generated with SWIG. The CoreRobotics modules included in the wrapper
are:

* - CRClock
* - CRTypes
* - CRMath
* - CRFrame
* - CRFrameEuler
* - CRFrameDh
* - CRRigidBody
* - CRManipulator
* - CRNoiseModel
* - CRNoiseGaussian
* - CRNoiseUniform
* - CRNoiseMixture
* - CRSensorLinear
* - CRInverseKinematics
* - CRNullSpace
* - CRHardLimits
* - CRSharedMemory
* - CRTrajectoryGenerator

The Matlab wrappers can be built by running CMake with `-Dmatlab=true` to
build the Matlab wrappers which are located at `matlab/lib`. To build all libraries
(C++, Python, and Matlab) run `cmake ../ -Dall=true`. This command will automatically
check if Python and Matlab are installed and build libraries accordingly. Two out of the
three libraries can also be built by specifing which ones, as in the command
`cmake ../ -Dmatlab=true -Dcpp=true` which will build C++ and Matlab libraries.

Since these wrappers are built using SWIG, if SWIG with Matlab support (Not part of the
SWIG master branch) is found on your system it will be used to generate the wrapper code.
If this is not the case then CMake will use the SWIG wrapper files it has stored for this
purpose. If SWIG is installed but it is desired to not use it the `noswig` flag can be set
to prevent SWIG from being used.

When building these wrappers on Windows car must be taken to use a compiler that is compatible with
your version of Matlab. The page at https://www.mathworks.com/support/sysreq/previous_releases.html
contains links to documents listing compatible compilers. The only free compiler supported appears
to be the MinGW compiler. Once the correct version of this compiler is installed run the CMake command
`cmake -G "MinGW Makefiles" ../ -Dmatlab=true` to build the Matlab wrappers, then use `mingw32-make`
to build the wrappers.

Examples of how to use the Matlab wrapper can be found in the examples provided
in the `matlab/util` directory.

\warning
* - The CRMath module has had the Forward Euler and 4th order Runge-Kutta integrators
removed for compatability.
* - The following modules have not been tested:
	* - Math
	* - FrameDh
	* - NoiseModel
	* - NoiseGaussian
	* - NoiseUniform
	* - NoiseMixture
	* - SensorLinear
	* - NullSpace
	* - HardLimits
*/


%include "eigen.i"

%module CoreRobotics
%{
#include "CoreRobotics.hpp"
#include "matlabHelpers.hpp"
%}

%include <typemaps.i>
%include <std_vector.i>
%include <std_shared_ptr.i>

%template(vectorMatrixXd) std::vector<Eigen::MatrixXd>;
%template(vectorVectorXd) std::vector<Eigen::VectorXd>;
%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)

%eigen_typemaps(Eigen::Matrix<double, 6, 1>)
%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(Eigen::Matrix3d)
%eigen_typemaps(Eigen::Matrix4d)
%eigen_typemaps(Eigen::Matrix<int, 6, 1>)

%include "Clock.hpp"
//%include "CRThread.hpp"
%include "Types.hpp"
%include "Conversion.hpp"
%include "Integration.hpp"
%include "Matrix.hpp"
%include "Frame.hpp"
%include "FrameEuler.hpp"
%include "FrameDh.hpp"
%include "RigidBody.hpp"
// %include "Manipulator.hpp"
%include "NoiseModel.hpp"
%include "NoiseGaussian.hpp"
%include "NoiseUniform.hpp"
%include "NoiseMixture.hpp"
//%include "SensorModel.hpp"
// %include "SensorLinear.hpp"
//%include "SensorProbabilistic.hpp"
//%include "MotionModel.hpp"
//%include "MotionLinear.hpp"
//%include "MotionProbabilistic.hpp"
// %include "InverseKinematics.hpp"
// %include "NullSpace.hpp"
// %include "HardLimits.hpp"
%include "SharedMemory.hpp"
// %include "TrajectoryGenerator.hpp"

%include "matlabHelpers.hpp"
