/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "KalmanFilter.hpp"

namespace cr {
namespace estimation {

//------------------------------------------------------------------------------
KalmanFilter::KalmanFilter(const model::MotionLG& i_motion,
				 		   const model::SensorLG& i_sensor,
				 		   const noise::Gaussian& i_state)
  : m_motion(i_motion), m_sensor(i_sensor), m_state(i_state) {}

//------------------------------------------------------------------------------
void KalmanFilter::step() {
  m_state = predict(m_action);
  m_state = correct(m_measurement);
}

//------------------------------------------------------------------------------
const noise::Gaussian& KalmanFilter::predict(const Eigen::VectorXd& i_u) {
  m_motion.setState(m_state);
  m_motion.setAction(i_u);
  m_motion.step();
  return m_motion.getState();
}

//------------------------------------------------------------------------------
const noise::Gaussian KalmanFilter::correct(const Eigen::VectorXd& i_z) {
  auto x = m_state.getParameters();
  m_sensor.setState(x.mean);
  m_sensor.step();
  const auto& e = i_z - m_sensor.getMeasurement().getParameters().mean;
  const auto& K = getKalmanGain();
  x.mean += K * e;
  const auto& KH = K * m_sensor.getParameters().m_H;
  x.cov = (Eigen::MatrixXd::Identity(KH.rows(), KH.cols()) - KH) * x.cov;
  noise::Gaussian g(x);
  return g;
}

//------------------------------------------------------------------------------
Eigen::MatrixXd KalmanFilter::getKalmanGain() {
  const auto& S = m_sensor.getParameters().m_H * m_state.getParameters().cov
     * m_sensor.getParameters().m_H.transpose() + m_sensor.getParameters().m_R;
  Eigen::MatrixXd Sinv = S.inverse();
  return m_state.getParameters().cov * m_sensor.getParameters().m_H * Sinv;
}

} // namespace estimation
} // namespace cr
