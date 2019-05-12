/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_SENSOR_LG_HPP_
#define CR_SENSOR_LG_HPP_

#include "Eigen/Dense"
#include "Sensor.hpp"
#include "SensorLinear.hpp"
#include "noise/Gaussian.hpp"

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
class SensorLG : public Sensor<noise::Gaussian, Eigen::VectorXd> {

public:
  //! Parameters
  class Parameters : public SensorLinear::Parameters {
  public:
    Parameters() = default;
    Parameters(std::size_t i_stateDim,
               std::size_t i_measDim,
               std::size_t i_noiseDim)
        : SensorLinear::Parameters(i_measDim, i_stateDim),
          m_R(Eigen::MatrixXd::Zero(i_measDim, i_measDim)){};
    virtual ~Parameters() = default;
    const Eigen::MatrixXd& R() const { return m_R; }
    void setR(const Eigen::MatrixXd& i_R ) { m_R = i_R; }
  private:
    Eigen::MatrixXd m_R; /** Measurement noise covariance */
  };

  //! Class constructor
  SensorLG(const Parameters &i_parameters,
           const Eigen::VectorXd &i_state,
           const double i_dt = 0.01);

  //! Class destructor
  virtual ~SensorLG() = default;

  CR_ASPECT_PARAMETER_MUTABLE(SensorLG::Parameters);

public:
  //! The prototype sensorCallback function.
  noise::Gaussian sensorCallback(double i_t, Eigen::VectorXd i_x) override;
};

} // namepsace model
} // namepsace cr

#endif
