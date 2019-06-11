/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "MinimumJerk.hpp"
#include "math/Matrix.hpp"
#include <iostream>
#include <math.h>

namespace cr {
namespace control {

//------------------------------------------------------------------------------
/*!
 This method computes the trajectory for the specified conditions.  When
 this method is called, the internal clock is reset to time = 0.\n

 \param[in]     i_x0 - initial state vector (position)
 \param[in]     i_v0 - initial state 1st derivative (velocity)
 \param[in]     i_a0 - initial state 2nd derivative (acceleration)
 \param[in]     i_xf - final state vector (position)
 \param[in]     i_vf - final state 1st derivative (velocity)
 \param[in]     i_af - final state 2nd derivative (acceleration)
 \param[in]     i_duration - duration of trajectory (s)
 \return        cr::Result indicator
 */
//------------------------------------------------------------------------------
core::Result MinimumJerk::Parameters::solve(const Eigen::VectorXd& i_x0,
                                            const Eigen::VectorXd& i_v0,
                                            const Eigen::VectorXd& i_a0,
                                            const Eigen::VectorXd& i_xf,
                                            const Eigen::VectorXd& i_vf,
                                            const Eigen::VectorXd& i_af, 
                                            double i_duration) {

  // indicator if solution is singular
  core::Result result =
      core::CR_RESULT_SUCCESS; // break the algorithm if it is singular

  // Set the internal time to the specified final time
  m_duration = i_duration;

  // Compute the inv(A) matrix
  double t = m_duration;
  Eigen::Matrix<double, 6, 6> Ainv;
  Ainv << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0,
      -10 / pow(t, 3), -6 / pow(t, 2), -3 / (2 * t), 10 / pow(t, 3),
      -4 / pow(t, 2), 1 / (2 * t), 15 / pow(t, 4), 8 / pow(t, 3),
      3 / (2 * pow(t, 2)), -15 / pow(t, 4), 7 / pow(t, 3), -1 / pow(t, 2),
      -6 / pow(t, 5), -3 / pow(t, 4), -1 / (2 * pow(t, 3)), 6 / pow(t, 5),
      -3 / pow(t, 4), 1 / (2 * pow(t, 3));

  // Compute the b matrix
  int n = i_x0.size();
  Eigen::Matrix<double, 6, Eigen::Dynamic> b;
  b.setZero(6, n);
  b << i_x0.transpose(), i_v0.transpose(), i_a0.transpose(), i_xf.transpose(),
      i_vf.transpose(), i_af.transpose();

  // Find the coefficient matrix
  m_A = Ainv * b;

  // return result
  return result;
}

//------------------------------------------------------------------------------
/*!
 This method computes the trajectory for the specified conditions.  When
 this method is called, the internal clock is reset to time = 0.\n

 \param[in]     i_wp0 - initial waypoint
 \param[in]     i_wpf - final waypoint
 \return        cr::Result indicator
 */
//------------------------------------------------------------------------------
core::Result MinimumJerk::Parameters::solve(const KinematicWaypoint& i_wp0, 
                                            const KinematicWaypoint& i_wpf) {
  // define the vectors
  Eigen::VectorXd x0 = i_wp0.position;
  Eigen::VectorXd v0 = i_wp0.velocity;
  Eigen::VectorXd a0 = i_wp0.acceleration;
  Eigen::VectorXd xf = i_wpf.position;
  Eigen::VectorXd vf = i_wpf.velocity;
  Eigen::VectorXd af = i_wpf.acceleration;
  double tf = i_wpf.time - i_wp0.time;

  // solve
  return solve(x0, v0, a0, xf, vf, af, tf);
}

//------------------------------------------------------------------------------
/*!
 This method computes the values of the trajectory at time i_t (s).\n

 \param[in]     i_t - the time at which to evaluate.
 \return        Waypoint trajectory structure.
 */
//------------------------------------------------------------------------------
KinematicWaypoint MinimumJerk::policyCallback(double i_t) {
  // Limit the defined time
  double t = i_t;
  if (t >= m_parameters.getDuration()) {
    t = m_parameters.getDuration();
  }
  if (t < 0) {
    t = 0;
  }

  // Define a waypoint
  KinematicWaypoint wp;
  wp.time = t;

  //! Velocity Coefficients
  Eigen::Matrix<double, 1, 5> vCoeff;
  vCoeff << 1.0, 2.0, 3.0, 4.0, 5.0;

  //! Acceleration Coefficients
  Eigen::Matrix<double, 1, 4> aCoeff;
  aCoeff << 2.0, 6.0, 12.0, 20.0;

  //! Acceleration Coefficients
  Eigen::Matrix<double, 1, 3> jCoeff;
  jCoeff << 6, 24, 60;

  //! Compute the polynomials in time
  Eigen::Matrix<double, 6, 1> T;
  T << pow(t, 0), pow(t, 1), pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);

  // Initialize the output values to zero
  int n = m_parameters.coefficients().cols();
  wp.position.setZero(n, 1);
  wp.velocity.setZero(n, 1);
  wp.acceleration.setZero(n, 1);
  wp.jerk.setZero(n, 1);

  // Now compute the output for each element
  for (int i = 0; i < n; i++) {

    // Get the coefficient vector
    Eigen::Matrix<double, 1, 6> X;
    X = m_parameters.coefficients().col(i).transpose();

    wp.position(i) = X * T; // pos
    wp.velocity(i) = vCoeff.cwiseProduct(X.tail(5)) * T.head(5);
    wp.acceleration(i) = aCoeff.cwiseProduct(X.tail(4)) * T.head(4);
    wp.jerk(i) = jCoeff.cwiseProduct(X.tail(3)) * T.head(3);
  }

  // return the struct
  return wp;
}

} // control
} // cr
