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


%include "eigen.i"

%module CoreRobotics
%{
#include "CRClock.hpp"
//#include "CRThread.hpp"
#include "CRTypes.hpp"
#include "CRMath.hpp"
#include "CRFrame.hpp"
#include "CRFrameEuler.hpp"
#include "CRFrameDh.hpp"
#include "CRRigidBody.hpp"
#include "CRManipulator.hpp"
#include "CRNoiseModel.hpp"
#include "CRNoiseGaussian.hpp"
#include "CRNoiseDirac.hpp"
#include "CRNoiseUniform.hpp"
#include "CRNoiseMixture.hpp"
//#include "CRSensorModel.hpp"
#include "CRSensorLinear.hpp"
//#include "CRSensorProbabilistic.hpp"
//#include "CRMotionModel.hpp"
//#include "CRMotionLinear.hpp"
//#include "CRMotionProbabilistic.hpp"
#include "CRInverseKinematics.hpp"
#include "CRNullSpace.hpp"
#include "CRHardLimits.hpp"
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

%include "CRClock.hpp"
//%include "CRThread.hpp"
%include "CRTypes.hpp"
%include "CRMath.hpp"
%include "CRFrame.hpp"
%include "CRFrameEuler.hpp"
%include "CRFrameDh.hpp"
%include "CRRigidBody.hpp"
%include "CRManipulator.hpp"
%include "CRNoiseModel.hpp"
%include "CRNoiseGaussian.hpp"
%include "CRNoiseDirac.hpp"
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
