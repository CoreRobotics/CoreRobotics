/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */


/*!
\page python Python Wrapper

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
*/

%module CoreRobotics

%{
#define SWIG_FILE_WITH_INIT
#include <memory>
#include <Python.h>

#include <cr/core>

// typdefs
typedef std::shared_ptr<cr::core::Loop> LoopPtr;
typedef std::shared_ptr<cr::core::Step> StepPtr;
%}

%include <eigen.i>
%include <std_string.i>
%include <std_shared_ptr.i>
%include <std_vector.i>
%include <typemaps.i>

// smart pointers
%shared_ptr(cr::core::Loop)
%shared_ptr(cr::core::Step)

/*
%{
namespace std {
template <typename T>
struct enable_shared_from_this {};
}
%}
*/

// %import std::enable_shared_from_this;
// %template(IStepSharedFromThis) std::enable_shared_from_this<cr::core::Step>;

// smart pointers
// %shared_ptr(Step);
// %template(IEnableStepPtr) std::enable_shared_from_this<cr::core::Step>;
// %template(IStepPtr) std::shared_ptr<cr::core::Step>;

/*
%template(DiscreteNoise) cr::noise::DistributionBase<unsigned>;
%template(ContinuousNoise) cr::noise::DistributionBase<Eigen::VectorXd>;

%template(DiscreteBase) cr::noise::Distribution<unsigned, DiscreteParameters>;
%template(GaussianBase) cr::noise::Distribution<Eigen::VectorXd, GaussianParameters>;
%template(GmmBase) cr::noise::Mixture<Eigen::VectorXd, Gaussian>;
%template(UniformBase) cr::noise::Distribution<Eigen::VectorXd, UniformParameters>;

%template(DynamicalSystemBase) cr::model::Motion<Eigen::VectorXd, Eigen::VectorXd>;
%template(SensorLinearBase) cr::model::Sensor<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>;
*/

// eigen
%template(IVectorMatrixXd) std::vector<Eigen::MatrixXd>;
%template(IVectorVectorXd) std::vector<Eigen::VectorXd>;
%eigen_typemaps(Eigen::VectorXd)
%eigen_typemaps(Eigen::MatrixXd)
%eigen_typemaps(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>)
%eigen_typemaps(Eigen::Matrix<double, 6, 1>)
%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(Eigen::Matrix3d)
%eigen_typemaps(Eigen::Matrix4d)
%eigen_typemaps(Eigen::Matrix<int, 6, 1>)

%init %{
import_array();
%}


// core
%include "core/Clock.hpp"
%include "core/Item.hpp"
%include "core/Loop.hpp"
%include "core/Step.hpp"
// %include "core/SharedMemory.hpp"
// %include "core/StepList.hpp"
// %include "core/Types.hpp"

// math
// %include "math/Conversion.hpp"
// %include "math/Integration.hpp"
// %include "math/Matrix.hpp"
// %include "math/Probability.hpp"

// physics
/*
%include "physics/Frame.hpp"
%include "physics/FrameDh.hpp"
%include "physics/FrameEuler.hpp"
%include "physics/RigidBody.hpp"

// world
%include "world/Link.hpp"
%include "world/Node.hpp"
%include "world/Origin.hpp"
%include "world/Robot.hpp"

// noise
%include "noise/Discrete.hpp"
%include "noise/Distribution.hpp"
%include "noise/Gaussian.hpp"
%include "noise/Gmm.hpp"
%include "noise/Mixture.hpp"
%include "noise/Uniform.hpp"

// signal
%include "signal/Log.hpp"
%include "signal/Signal.hpp"
%include "signal/Slot.hpp"
%include "signal/Utility.hpp"

// model
%include "model/DynamicalSystem.hpp"
%include "model/Motion.hpp"
%include "model/MotionLinear.hpp"
%include "model/Sensor.hpp"
%include "model/SensorLinear.hpp"

// control
%include "control/TrajectoryGenerator.hpp"
*/
