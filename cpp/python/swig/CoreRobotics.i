%include "eigen.i"

%module CoreRobotics
%{
#define SWIG_FILE_WITH_INIT
#include <Python.h>

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
