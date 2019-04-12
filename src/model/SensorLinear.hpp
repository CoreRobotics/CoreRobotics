/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_SENSOR_LINEAR_HPP_
#define CR_SENSOR_LINEAR_HPP_

#include "Eigen/Dense"
#include "Sensor.hpp"

namespace cr {
namespace model {

//------------------------------------------------------------------------------
/*!
 \class SensorLinear
 \ingroup model

 \brief This class implements a linear sensor model.

 \details
 ## Description
 SensorLinear implements a snesor model from a supplied linear
 callback function.  Specifically, SensorLinear sets up a container
 for the continuous model

 \f[
 z = Cx + Du
 \f]

 where \f$x\f$ is the state, \f$u\f$ is the input, \f$t\f$ is time, \f$z\$ is
 the measurement, and \f$k\f$ is a discrete sampling index.

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006.
 \n\n
 */
//------------------------------------------------------------------------------
class SensorLinear
  : public Sensor<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd> {

public:

  //! Class constructor
  SensorLinear(const Eigen::MatrixXd& i_C, const Eigen::MatrixXd& i_D,
               const Eigen::VectorXd& i_x0, const Eigen::VectorXd& i_u0);
    
  //! Class destructor
  virtual ~SensorLinear() = default;

public:

  //! The prototype sensorCallback function.
  virtual Eigen::VectorXd sensorCallback(double i_t, Eigen::VectorXd i_x,
    Eigen::VectorXd i_u) override;

public:

  //! Set the dynamics and input matrices
  void setObservation(const Eigen::MatrixXd& i_C, const Eigen::MatrixXd& i_D) {
    m_C = i_C;
    m_D = i_D;
  }

protected:

  //! Observation Matrix
  Eigen::MatrixXd m_C;

  //! Feedthrough Matrix
  Eigen::MatrixXd m_D;
};

}  // namepsace model
}  // namepsace cr

#endif
