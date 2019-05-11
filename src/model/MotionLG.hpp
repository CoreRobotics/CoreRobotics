/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_MOTION_LG_HPP_
#define CR_MOTION_LG_HPP_

#include "Motion.hpp"
#include "noise/Gaussian.hpp"
#include "Eigen/Dense"

namespace cr {
namespace model {

//! Motion linear Gaussian paramter structure
struct MotionLGParameters {
  MotionLGParameters() = default;
  virtual ~MotionLGParameters() = default;

  //! Initializes the multivariate standard uniform
  MotionLGParameters(std::size_t i_stateDim, std::size_t i_actionDim,
    std::size_t i_noiseDim)
      : m_A(Eigen::MatrixXd::Zero(i_stateDim, i_stateDim)), 
        m_B(Eigen::MatrixXd::Zero(i_stateDim, i_actionDim)),
        m_C(Eigen::MatrixXd::Zero(i_stateDim, i_noiseDim)),
        m_Q(Eigen::MatrixXd::Zero(i_noiseDim, i_noiseDim)) {};

  Eigen::MatrixXd m_A;  /** Dynamics matrix */
  Eigen::MatrixXd m_B;  /** Input matrix */
  Eigen::MatrixXd m_C;  /** Process noise matrix */
  Eigen::MatrixXd m_Q;  /** Process noise covariance */
};

//------------------------------------------------------------------------------
/*!
 \class MotionLG
 \ingroup model

 \brief This class implements a linear Gaussian motion model.

 \details
 ## Description
 MotionLG implements a motion model from a supplied dynamics callback
 function.  Specifically, MotionLinear sets up a container for the continuous
 time model

 \f[
 \dot{x} = Ax + Bu + Cw
 \f]

 or

 \f[
 x_{k+1} = A x_k + B u_k + C w_k
 \f]

 where \f$x\f$ is the state vector, \f$u\f$ is the input vector, \f$w\f$ is 
 the Gaussian noise vector, and \f$k\f$ is a discrete sampling index.

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006.
 \n\n
 */
//------------------------------------------------------------------------------
class MotionLG
  : public Motion<noise::Gaussian, Eigen::VectorXd, MotionLGParameters> {

public:
  //! Class constructor
  MotionLG(const MotionLGParameters &i_parameters,
           const noise::Gaussian &i_state, 
           const Eigen::VectorXd &i_action,
           const double i_dt = 0.01);

  //! Class destructor
  virtual ~MotionLG() = default;

public:
  //! The prototype motionCallback function.
  noise::Gaussian motionCallback(double i_t, noise::Gaussian i_x,
                                 Eigen::VectorXd i_u) override;
};

} // namepsace model
} // namepsace cr

#endif
