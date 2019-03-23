/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef TrajectoryGenerator_hpp
#define TrajectoryGenerator_hpp

#include "Eigen/Dense"
#include "core/Clock.hpp"
#include "core/Types.hpp"

namespace cr {
namespace control {

//! Structure defining a waypoint (i.e. the output)
struct Waypoint {
  double time;
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd jerk;
};

//------------------------------------------------------------------------------
/*!
 \class TrajectoryGenerator
 \ingroup control

 \brief This class provides methods for generating minimum jerk
 trajectories from initial and final conditions.

 \details
 ## Description
 TrajectoryGenerator implements the minimum-jerk trajectory generation
 technique from a set of initial condititions, final conditions, and their
 1st and 2nd derivatives.


 These methods are available with the trajectory generator:
 - TrajectoryGenerator::solve computes the minimum jerk trajectory for
 the specified initial and final conditions and stores the representation
 of the trajectory internally.
 - TrajectoryGenerator::step computes the values of the trajectory for
 a specified time (if specified) or for the time elapsed since the solve
 method was called (if time is not specified).

 ## Example
 This example demonstrates use of the TrajectoryGenerator class.
 \include example_TrajectoryGenerator.cpp

 ## References
 [1] N. Hogan, "Adaptive control of mechanical impedance by coactivation of
     antagonist muscles," IEEE Trans. on Automatic Control AC-29: 681-690,
     1984. \n\n

 */
//------------------------------------------------------------------------------
class TrajectoryGenerator {

public:
  //! Class constructor
  TrajectoryGenerator();

public:
  //! Solve for the coefficients needed to achieve the trajectory
  core::Result solve(Eigen::VectorXd i_x0, Eigen::VectorXd i_v0,
                     Eigen::VectorXd m_a0, Eigen::VectorXd m_xf,
                     Eigen::VectorXd m_vf, Eigen::VectorXd m_af, double i_tf);

  //! Solve for the coefficients needed to achieve the trajectory
  core::Result solve(Waypoint i_wp0, Waypoint i_wpf);

  //! Get the trajectory at time t
  Waypoint step(double i_t);

  //! Step the next trajectory reference
  Waypoint step(void);

protected:
  //! Final time
  double m_tf = 1.0;

  //! Polynomial coefficient matrix
  Eigen::Matrix<double, 6, Eigen::Dynamic> m_X;

  //! An internal clock for keeping track of time
  core::Clock m_timer;
};

} // namepsace control
} // namepsace cr

#endif
