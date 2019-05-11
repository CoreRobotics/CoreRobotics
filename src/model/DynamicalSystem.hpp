/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_DYNAMICAL_SYSTEM_HPP_
#define CR_DYNAMICAL_SYSTEM_HPP_

#include "Eigen/Dense"
#include "Motion.hpp"

namespace cr {
namespace model {

//! Dynamical system type
enum SystemType {
  //! motionCallback returns next state \f$x_{k+1}\f$
  DISCRETE_TIME,
  //! motionCallback returns time rate of change of state \f$\dot{x}\f$
  CONTINUOUS_TIME,
};

//------------------------------------------------------------------------------
/*!
 \class DynamicalSystem
 \ingroup model

 \brief This class implements a dynamical system model.

 \details
 ## Description
 DynamicalSystem implements a motion model from a supplied dynamics
 callback function.  Specifically, DynamicalSystem sets up a container
 for the continuous time model

 \f[
 \dot{x} = f(t, x, u)
 \f]

 or the discrete-time model

 \f[
 x_{k+1} = f(t_k, x_k, u_k)
 \f]

 where \f$x\f$ is the state vector, \f$u\f$ is the input vector,
 \f$t\f$ is time, and \f$k\f$ is a discrete sampling index.

 Use the ModelType parameter to select which callback is implemented.

  To use this class, users must derive
 - `StateType motionCallback(double i_t, StateType i_x, ActionType i_u)`

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006.
 \n\n
 */
//------------------------------------------------------------------------------
template <typename ParameterType = void *>
class DynamicalSystem
    : public Motion<Eigen::VectorXd, Eigen::VectorXd, ParameterType> {

public:
  //! Class constructor
  DynamicalSystem(const ParameterType &i_parameters,
                  const Eigen::VectorXd &i_state,
                  const Eigen::VectorXd &i_action, const double i_dt = 0.01,
                  const SystemType &i_type = CONTINUOUS_TIME)
      : Motion<Eigen::VectorXd, Eigen::VectorXd, ParameterType>(
            i_parameters, i_state, i_action, i_dt),
        m_systemType(i_type){};

  //! Class destructor
  virtual ~DynamicalSystem() = default;

public:
  //! The prototype motionCallback function must be implemented.
  virtual Eigen::VectorXd motionCallback(double i_t, Eigen::VectorXd i_x,
                                         Eigen::VectorXd i_u) override = 0;

  //! This function steps the callback and updates the state.
  void step() override {
    if (m_systemType == DISCRETE_TIME) {
      this->m_state =
          (this->m_motion_fcn)(this->m_time, this->m_state, this->m_action);
    } else if (m_systemType == CONTINUOUS_TIME) {
      this->m_state = math::Integration::rungeKuttaStep(
          this->m_motion_fcn, this->m_time, this->m_state, this->m_action,
          this->m_dt);
    }
    this->m_time += this->m_dt;
  };

public:
  //! Set the system type
  void setSystemType(const SystemType &i_type) { m_systemType = i_type; }

  //! Get the system type
  const SystemType &getSystemType() { return m_systemType; }

protected:
  //! system type
  SystemType m_systemType;
};

} // namepsace model
} // namepsace cr

#endif
