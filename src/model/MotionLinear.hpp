/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_MOTION_LINEAR_HPP_
#define CR_MOTION_LINEAR_HPP_

#include "DynamicalSystem.hpp"
#include "Eigen/Dense"

namespace cr {
namespace model {

//! Motion linear paramter structure
struct MotionLinearParameters {
  MotionLinearParameters() = default;
  virtual ~MotionLinearParameters() = default;

  //! Initializes the multivariate standard uniform
  MotionLinearParameters(std::size_t i_stateDim, std::size_t i_actionDim)
      : m_A(Eigen::MatrixXd::Zero(i_stateDim, i_stateDim)),
        m_B(Eigen::MatrixXd::Zero(i_stateDim, i_actionDim)){};

  Eigen::MatrixXd m_A; /** Dynamics matrix */
  Eigen::MatrixXd m_B; /** Input matrix */
};

//------------------------------------------------------------------------------
/*!
 \class MotionLinear
 \ingroup model

 \brief This class implements a linear motion model.

 \details
 ## Description
 MotionLinear implements a motion model from a supplied dynamics callback
 function.  Specifically, MotionLinear sets up a container for the continuous
 time model

 \f[
 \dot{x} = Ax + Bu
 \f]

 or

 \f[
 x_{k+1} = A x_k + B u_k
 \f]

 where \f$x\f$ is the state vector, \f$u\f$ is the input vector,
 and \f$k\f$ is a discrete sampling index.

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006.
 \n\n
 */
//------------------------------------------------------------------------------
class MotionLinear : public DynamicalSystem<MotionLinearParameters> {

public:
  //! Class constructor
  MotionLinear(const MotionLinearParameters &i_parameters,
               const Eigen::VectorXd &i_state, const Eigen::VectorXd &i_action,
               const double i_dt = 0.01,
               const SystemType i_type = CONTINUOUS_TIME);

  //! Class destructor
  virtual ~MotionLinear() = default;

public:
  //! The prototype motionCallback function.
  Eigen::VectorXd motionCallback(double i_t, Eigen::VectorXd i_x,
                                 Eigen::VectorXd i_u) override;
};

} // namepsace model
} // namepsace cr

#endif
