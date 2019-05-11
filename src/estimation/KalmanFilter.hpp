/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_KALMAN_FILTER_HPP_
#define CR_KALMAN_FILTER_HPP_

#include "Eigen/Dense"
#include "core/Step.hpp"
#include "model/MotionLG.hpp"
#include "model/SensorLG.hpp"
#include "noise/Gaussian.hpp"

namespace cr {
namespace estimation {

//------------------------------------------------------------------------------
/*!
 \class KalmanFilter
 \ingroup estimation

 \brief This class provides methods for solving continuous and discrete
 time state estimation problems using a kalman filter.

 \details
 ## Description
 KalmanFilter is a module with implements the standard discrete and
 continuous time Kalman filters. This module expects a continuous time
 system in the form,

 \f$ \dot{x}=Ax+Bu+v \f$

 \f$ z=Cx+w \f$,

 where \f$x\f$ is the state vector, \f$z\f$ is a measurement, and \f$v\f$
 and \f$w\f$ are zero mean gaussian random vectors with covariance matricies
 \f$Q\f$ and \f$R\f$ respectively.

 In discrete time the modules expects a system,

 \f$ x_{t+1}=Ax_t+Bu_t+v \f$

 \f$ z_t=Cx_t \f$,

 where all the variable are defined in the same way as the continuous time
 version.

 ## References

 [1] Wikipedia: "Kalman filter",
 https://en.wikipedia.org/wiki/Kalman_filter 2017.\n\n

 */
//------------------------------------------------------------------------------
class KalmanFilter : public core::Step, public core::Item {

public:
  //! Class constructor
  KalmanFilter(const model::MotionLG &i_motion, const model::SensorLG &i_sensor,
               const noise::Gaussian &i_state);

  //! Class destructor
  virtual ~KalmanFilter() = default;

  //! This function steps the callback and updates the state.
  void step() override;

  //! Set the state estimate
  void setState(const noise::Gaussian &i_x) { m_state = i_x; }

  //! Get the state estimate
  const noise::Gaussian &getState() { return m_state; }

  //! Prediction step
  void predict(const Eigen::VectorXd &i_u, noise::Gaussian &o_x);

  //! Correction step
  void correct(const Eigen::VectorXd &i_z, noise::Gaussian &o_x);

  //! Get the Kalman Gain
  Eigen::MatrixXd getKalmanGain();

  //! Set the observed sensor measurement
  void setMeasurement(Eigen::VectorXd i_z) { m_measurement = i_z; }

  //! Set the applied motion action
  void setAction(Eigen::VectorXd i_u) { m_action = i_u; }

  //---------------------------------------------------------------------
  // Protected Members
protected:
  //! The linear Gaussian motion model
  model::MotionLG m_motion;

  //! The linear Gaussian sensor model
  model::SensorLG m_sensor;

  //! The state estimate
  noise::Gaussian m_state;

  //! Sensor measurement
  Eigen::VectorXd m_measurement;

  //! Motion action
  Eigen::VectorXd m_action;
};

} // namespace estimation
} // namespace cr

#endif
