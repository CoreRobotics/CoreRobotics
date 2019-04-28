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
class MotionLinear : public DynamicalSystem {

public:
  //! Class constructor
  MotionLinear(const Eigen::MatrixXd &i_A, const Eigen::MatrixXd &i_B,
               const Eigen::VectorXd &i_x0, const Eigen::VectorXd &i_u0,
               const double i_dt = 0.01, DynamicalSystem::ModelType i_type =
                                             DynamicalSystem::CONTINUOUS_TIME);

  //! Class destructor
  virtual ~MotionLinear() = default;

public:
  //! The prototype motionCallback function.
  virtual Eigen::VectorXd motionCallback(double i_t, Eigen::VectorXd i_x,
                                         Eigen::VectorXd i_u) override;

public:
  //! Set the dynamics and input matrices
  void setDynamics(const Eigen::MatrixXd &i_A, const Eigen::MatrixXd &i_B) {
    m_A = i_A;
    m_B = i_B;
  };

protected:
  //! Dynamics Matrix
  Eigen::MatrixXd m_A;

  //! Input Matrix
  Eigen::MatrixXd m_B;
};

} // namepsace model
} // namepsace cr

#endif
