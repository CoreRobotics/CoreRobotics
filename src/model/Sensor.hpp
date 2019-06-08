/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_MODEL_SENSOR_HPP_
#define CR_MODEL_SENSOR_HPP_

#include "Eigen/Dense"
#include "aspect/Measurement.hpp"
#include "aspect/Parameter.hpp"
#include "aspect/State.hpp"
#include "aspect/Temporal.hpp"
#include "core/Item.hpp"
#include "core/Step.hpp"

namespace cr {
namespace model {

namespace ph = std::placeholders;

//------------------------------------------------------------------------------
/*!
 \class Sensor
 \ingroup model

 \brief This class implements a sensor model from a supplied observation callback
 function.  Specifically, MotionModel sets up a container for the discrete-time 
 set of equations

 \f[
 z_k = h(t_k, x_k)
 \f]

 where \f$x\f$ is state, \f$t\f$ is time, \f$z\f$ is measurement, and \f$k\f$
 is a discrete sampling index.

 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n

 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006.
 \n\n
 */
//------------------------------------------------------------------------------
template <typename MeasurementType, typename StateType>
class Sensor : public core::Step, public core::Item {

public:
  //! Class constructor
  Sensor(const StateType &i_state,
         const double i_dt = 0.01)
      : m_state(i_state), m_dt(i_dt) {}

  //! Class destructor
  virtual ~Sensor() = default;

  CR_ASPECT_STATE_WRITE(StateType);

  CR_ASPECT_MEASUREMENT_WRITE(MeasurementType);

  CR_ASPECT_TEMPORAL_DISCRETIZED

public:
  //! The prototype sensorCallback function must be implemented.
  virtual MeasurementType sensorCallback(double i_t, StateType i_x) = 0;

  ///! This function steps the callback and updates the state.
  virtual void step() {
    m_measurement = this->m_sensor_fcn(m_time, m_state);
    m_time += m_dt;
  }

protected:
  //! bound callback function
  std::function<MeasurementType(double, StateType)> m_sensor_fcn =
      std::bind(&Sensor::sensorCallback, this, ph::_1, ph::_2);
};

} // namepsace model
} // namepsace cr

#endif
