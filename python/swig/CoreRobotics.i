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
 \version 0.0
 
 */
//=====================================================================


/*!
\page python Python Wrapper

A Python wrapper for a portion of the CoreRobotics library has been
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

The Python wrappers can be built by running CMake with `-Dpython=true` to
build the Python wrappers which are located at `python/lib`. To build all libraries
(C++, Python, and Matlab) run `cmake ../ -Dall=true`. This command will automatically
check if Python and Matlab are installed and build libraries accordingly.
These commands use the latest version of Python available on your system.
To compile with a specific version use `cmake ../ -Dpython=2` to compile with Python 2 etc.
This option can also be used with the `all` flag to compile both the C++ library, and the Python
library with a specified version, for example `cmake ../ -Dall=true -Dpython=2` will attempt to
build C++, Python 2 and Matlab libraries, checking if Python and Matlab exist. Two out of the three
libraries can also be built by specifing which ones, as in the command `cmake ../ -Dpython=true -Dcpp=true`
which will build C++ and Python libraries.

Since these wrappers are built using SWIG, if SWIG is found on your system it will be used to
generate the wrapper code. If SWIG is not found on your system then CMake will use the SWIG wrapper
files it has stored for this purpose. If SWIG is installed but it is desired to not use it the `noswig`
flag can be set to prevent SWIG from being used.

Examples of how to use the Python wrapper can be found in the examples provided
in the `python/util` directory.

\warning
* - The CRClock module does not work as expected.
* - The CRMath module has had the Forward Euler and 4th order Runge-Kutta integrators
removed for compatability.
* - The following modules have not been tested:
	* - CRNoiseModel
	* - CRNoiseGaussian
	* - CRNoiseUniform
	* - CRNoiseMixture
	* - CRSensorLinear
*/


%include "eigen.i"

%module CoreRobotics
%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>

#include "CoreRobotics.hpp"
%}

%include <typemaps.i>
%include <std_vector.i>

%template(vectorMatrixXd) std::vector<Eigen::MatrixXd>;
%template(vectorVectorXd) std::vector<Eigen::VectorXd>;
%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)

%eigen_typemaps(Eigen::Matrix<double, 6, 1>)
%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(Eigen::Matrix4d)
%eigen_typemaps(Eigen::Matrix<int, 6, 1>)

%init %{
import_array();
%}

%include "CRClock.hpp"
//%include "CRThread.hpp"
%include "CRTypes.hpp"
%include "CRConversion.hpp"
%include "CRIntegration.hpp"
%include "CRMatrix.hpp"
%include "CRFrame.hpp"
%include "CRFrameEuler.hpp"
%include "CRFrameDh.hpp"
%include "CRRigidBody.hpp"
%include "CRManipulator.hpp"
%include "CRNoiseModel.hpp"
%include "CRNoiseGaussian.hpp"
%include "CRNoiseUniform.hpp"
%include "CRNoiseMixture.hpp"
//%include "CRSensorModel.hpp"
%include "CRSensorLinear.hpp"
//%include "CRSensorProbabilistic.hpp"
//%include "CRMotionModel.hpp"
//%include "CRMotionLinear.hpp"
//%include "CRMotionProbabilistic.hpp"
%include "CRInverseKinematics.hpp"
%include "CRNullSpace.hpp"
%include "CRHardLimits.hpp"
%include "CRSharedMemory.hpp"
%include "CRTrajectoryGenerator.hpp"
