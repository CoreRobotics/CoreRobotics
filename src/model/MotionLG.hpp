/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_MODEL_MOTION_LG_HPP_
#define CR_MODEL_MOTION_LG_HPP_

#include "Eigen/Dense"
#include "Motion.hpp"
#include "MotionLinear.hpp"
#include "noise/Gaussian.hpp"

namespace cr {
namespace model {

//------------------------------------------------------------------------------
/*!
 \class MotionLG
 \ingroup model

 \brief This class implements a linear Gaussian motion model from a supplied 
 dynamics callback function.  Specifically, MotionLinear sets up a container 
 for the continuous time model

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
class MotionLG : public Motion<noise::Gaussian, Eigen::VectorXd> {

public:
  //! Motion linear Gaussian paramter structure
  class Parameters : public MotionLinear::Parameters {
  public: 
    Parameters() = default;
    Parameters(std::size_t i_stateDim, std::size_t i_actionDim,
                      std::size_t i_noiseDim)
        : MotionLinear::Parameters(i_stateDim, i_actionDim),
          m_C(Eigen::MatrixXd::Zero(i_stateDim, i_noiseDim)),
          m_Q(Eigen::MatrixXd::Zero(i_noiseDim, i_noiseDim)){};
    virtual ~Parameters() = default;
    const Eigen::MatrixXd& C() const { return m_C; }
    const Eigen::MatrixXd& Q() const { return m_Q; }
    void setC(const Eigen::MatrixXd& i_C ) { m_C = i_C; }
    void setQ(const Eigen::MatrixXd& i_Q ) { m_Q = i_Q; }
  private:
    Eigen::MatrixXd m_C; /** Process noise matrix */
    Eigen::MatrixXd m_Q; /** Process noise covariance */
  };

  //! Class constructor
  MotionLG(const Parameters &i_parameters,
           const noise::Gaussian &i_state,
           const Eigen::VectorXd &i_action,
           const double i_dt = 0.01);

  //! Class destructor
  virtual ~MotionLG() = default;

  CR_ASPECT_PARAMETER_MUTABLE(MotionLG::Parameters);

public:
  //! The prototype motionCallback function.
  noise::Gaussian motionCallback(double i_t, noise::Gaussian i_x,
                                 Eigen::VectorXd i_u) override;
};

} // namepsace model
} // namepsace cr

#endif
