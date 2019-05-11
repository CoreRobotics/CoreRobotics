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

//! Sensor linear paramter structure
struct SensorLinearParameters {
  SensorLinearParameters() = default;
  virtual ~SensorLinearParameters() = default;

  //! Initializes the multivariate standard uniform
  SensorLinearParameters(std::size_t i_stateDim, std::size_t i_measDim)
      : m_H(Eigen::MatrixXd::Zero(i_measDim, i_stateDim)){};

  Eigen::MatrixXd m_H; /** Observation matrix */
};

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
 z = Hx
 \f]

 where \f$x\f$ is the state, \f$t\f$ is time, \f$z\$ is the measurement, and
\f$k\f$ is a discrete sampling index.

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006.
 \n\n
 */
//------------------------------------------------------------------------------
class SensorLinear
    : public Sensor<Eigen::VectorXd, Eigen::VectorXd, SensorLinearParameters> {

public:
  //! Class constructor
  SensorLinear(const SensorLinearParameters &i_parameters,
               const Eigen::VectorXd &i_state, const double i_dt = 0.01);

  //! Class destructor
  virtual ~SensorLinear() = default;

public:
  //! The prototype sensorCallback function.
  virtual Eigen::VectorXd sensorCallback(double i_t,
                                         Eigen::VectorXd i_x) override;
};

} // namepsace model
} // namepsace cr

#endif
