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

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006.
 \n\n
 */
//------------------------------------------------------------------------------
class DynamicalSystem
  : public Motion<Eigen::VectorXd, Eigen::VectorXd> {
  
public:

  //! Motion model type
  enum ModelType {
    DISCRETE_TIME,
    CONTINUOUS_TIME,
  };

public:

  //! Class constructor
  DynamicalSystem(const Eigen::VectorXd& i_x0, const Eigen::VectorXd& i_u0,
    const double i_dt = 0.01, ModelType i_type = CONTINUOUS_TIME);
    
  //! Class destructor
  virtual ~DynamicalSystem() = default;
  
public:

  //! The prototype motionCallback function must be implemented.
  virtual Eigen::VectorXd motionCallback(double i_t, Eigen::VectorXd i_x,
    Eigen::VectorXd i_u) override = 0;

  ///! This function steps the callback and updates the state.
  virtual void step() override;

public:
  
  //! Set the model type
  void setType(ModelType i_type) { m_type = i_type; }
  
  //! Get the model type
  ModelType getType() { return m_type; }

protected:

  //! model type
  ModelType m_type;
};

} // namepsace model
} // namepsace cr

#endif
