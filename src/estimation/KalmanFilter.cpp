/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "KalmanFilter.hpp"

namespace cr {
namespace estimation {

//------------------------------------------------------------------------------
KalmanFilter::KalmanFilter(const model::MotionLG &i_motion,
                           const model::SensorLG &i_sensor,
                           const noise::Gaussian &i_state)
    : m_motion(i_motion), m_sensor(i_sensor), m_state(i_state) {}

//------------------------------------------------------------------------------
void KalmanFilter::step() {
  predict(m_action, m_state);
  correct(m_measurement, m_state);
}

//------------------------------------------------------------------------------
void KalmanFilter::predict(const Eigen::VectorXd &i_u, noise::Gaussian &o_x) {
  m_motion.setState(o_x);
  m_motion.setAction(i_u);
  m_motion.step();
  o_x = m_motion.getState();
}

//------------------------------------------------------------------------------
void KalmanFilter::correct(const Eigen::VectorXd &i_z, noise::Gaussian &o_x) {
  auto gp = o_x.getParameters();
  m_sensor.setState(gp.mean);
  m_sensor.step();
  const auto &e = i_z - m_sensor.getMeasurement().getParameters().mean;
  const auto &K = getKalmanGain();
  gp.mean += K * e;
  const auto &KH = K * m_sensor.getParameters().m_H;
  gp.cov = (Eigen::MatrixXd::Identity(KH.rows(), KH.cols()) - KH) * gp.cov;
  o_x.setParameters(gp);
}

//------------------------------------------------------------------------------
Eigen::MatrixXd KalmanFilter::getKalmanGain() {
  const auto &S = m_sensor.getParameters().m_H * m_state.getParameters().cov *
                      m_sensor.getParameters().m_H.transpose() +
                  m_sensor.getParameters().m_R;
  Eigen::MatrixXd Sinv = S.inverse();
  return m_state.getParameters().cov * m_sensor.getParameters().m_H * Sinv;
}

} // namespace estimation
} // namespace cr
