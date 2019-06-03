/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_MINIMUM_JERK_HPP_
#define CR_MINIMUM_JERK_HPP_

#include "Eigen/Dense"
#include "aspect/Parameter.hpp"
#include "core/Types.hpp"
#include "control/TrajectoryGenerator.hpp"

namespace cr {
namespace control {

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
class MinimumJerk : public TrajectoryGenerator<Waypoint> {

public:
  //! Parameters
  class Parameters {
  public:
    Parameters() = default;
    Parameters(std::size_t i_actionDim)
        : m_A(Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, i_actionDim)) {};
    virtual ~Parameters() = default;

    //! Return the coefficient matrix
    const Eigen::MatrixXd coefficients() const { return m_A; }

    //! Get the durationn
    double getDuration() { return m_duration; }

    //! Solve for the coefficients needed to achieve the trajectory
    core::Result solve(const Eigen::VectorXd& i_x0, const Eigen::VectorXd& i_v0,
                       const Eigen::VectorXd& i_a0, const Eigen::VectorXd& i_xf,
                       const Eigen::VectorXd& i_vf, const Eigen::VectorXd& i_af,
                       double i_duration);

    //! Solve for the coefficients needed to achieve the trajectory
    core::Result solve(const Waypoint& i_wp0, const Waypoint& i_wpf);

  protected:
    /** Polynomial coefficient matrix */
    Eigen::Matrix<double, 6, Eigen::Dynamic> m_A;

    /** Duration of trajectory */
    double m_duration = 1.0;
  };

  //! Class constructor
  MinimumJerk(const Parameters &i_parameters,
              const Waypoint& i_action)
    : TrajectoryGenerator<Waypoint>(i_action), m_parameters(i_parameters) {}

  //! Destructor
  virtual ~MinimumJerk() = default;

  //! Factory
  static core::StepPtr create(const Parameters &i_parameters,
                              const Waypoint& i_action) {
    return core::StepPtr(new MinimumJerk(i_parameters, i_action));
  }

  CR_ASPECT_PARAMETER_MUTABLE(MinimumJerk::Parameters);

public:

  //! Callback for the trajectory.
  Waypoint policyCallback(double i_t) override;
  
};

} // namepsace control
} // namepsace cr

#endif
