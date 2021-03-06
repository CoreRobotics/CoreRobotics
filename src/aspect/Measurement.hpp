/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef CR_ASPECT_MEASUREMENT_HPP_
#define CR_ASPECT_MEASUREMENT_HPP_

/*!
  \def CR_ASPECT_MEASUREMENT_READ(MeasurementType)
  \ingroup aspect
  Classes that have a read only measurement aspect should include this macro.

  \brief This macro adds class members.
  \param MeasurementType  The measurement data type.
*/
#define CR_ASPECT_MEASUREMENT_READ(MeasurementType) \
public: \
  const MeasurementType& getMeasurement() const { return m_measurement; } \
protected: \
  MeasurementType m_measurement;

/*!
  \def CR_ASPECT_MEASUREMENT_WRITE(MeasurementType)
  \ingroup aspect
  Classes that have a read/write measurement aspect should include this macro.

  \brief This macro adds class members.
  \param MeasurementType  The measurement data type.
*/
#define CR_ASPECT_MEASUREMENT_WRITE(MeasurementType) \
CR_ASPECT_MEASUREMENT_READ(MeasurementType) \
public: \
  void setMeasurement(const MeasurementType& i_z) { m_measurement = i_z; }

/*!
  \def CR_ASPECT_MEASUREMENT_MUTABLE(MeasurementType)
  \ingroup aspect
  Classes that have a mutable measurement aspect should include this macro.

  \brief This macro adds class members.
  \param MeasurementType  The measurement data type.
*/
#define CR_ASPECT_MEASUREMENT_MUTABLE(MeasurementType) \
CR_ASPECT_MEASUREMENT_WRITE(MeasurementType) \
public: \
  MeasurementType* measurement() { return &m_measurement; }

#endif
