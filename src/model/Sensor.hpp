/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_SENSOR_HPP_
#define CR_SENSOR_HPP_

#include "Eigen/Dense"
#include "core/Item.hpp"
#include "core/Step.hpp"

namespace cr {
namespace model {

namespace ph = std::placeholders;

//------------------------------------------------------------------------------
/*!
 \class Sensor
 \ingroup model

 \brief This class implements a sensor model.

 \details
 ## Description
 Sensor implements a sesnor model from a supplied observation callback function.
 Specifically, MotionModel sets up a container for the discrete-time model

 \f[
 z_k = h(t_k, x_k)
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
template <typename MeasurementType, typename StateType, typename ParameterType>
class Sensor : public core::Step, public core::Item {

public:
  //! Class constructor
  Sensor(const ParameterType &i_parameters,
         const StateType &i_state,
         const double i_dt = 0.01)
    : m_parameters(i_parameters), m_state(i_state), m_dt(i_dt) {}

  //! Class destructor
  virtual ~Sensor() = default;

public:
  //! The prototype sensorCallback function must be implemented.
  virtual MeasurementType sensorCallback(double i_t, StateType i_x) = 0;

  ///! This function steps the callback and updates the state.
  virtual void step() {
    m_measurement = this->m_sensor_fcn(m_time, m_state);
    m_time += m_dt;
  }

public:

  //! Get the measurement vector (z)
  MeasurementType getMeasurement() { return m_measurement; }

  //! Set the state vector (x)
  void setState(const StateType &i_x) { m_state = i_x; }

  //! Get the state vector (x)
  StateType getState() { return m_state; }

  //! Set the parameters that describe the distribution.
  void setParameters(const ParameterType &i_parameters) {
    m_parameters = i_parameters;
  }

  //! Get the parameters that descrive the distribution.
  const ParameterType &getParameters() { return m_parameters; }

  //! Get the parameters that descrive the distribution.
  ParameterType *parameters() { return *m_parameters; }

  //! Set the time step (s)
  void setTimeStep(const double i_timeStep) { m_dt = i_timeStep; }

  //! Get the time step (s)
  double getTimeStep() { return m_dt; }

  //! Get the model time (s)
  double getTime() { return m_time; }

protected:
  //! bound callback function
  std::function<MeasurementType(double, StateType)> m_sensor_fcn =
      std::bind(&Sensor::sensorCallback, this, ph::_1, ph::_2);

  //! System parameters
  ParameterType m_parameters;

  //! Dynamic state of the system (x)
  StateType m_state;

  //! Sample rate (s)
  double m_dt;

  //! Current time (s)
  double m_time = 0.0;

  //! Sensor observation (z)
  MeasurementType m_measurement;
};

} // namepsace model
} // namepsace cr

#endif
