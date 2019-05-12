/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

//! Classes that maintain a time parameter should include this macro
#define CR_ASPECT_TEMPORAL \
public:\
  void setTimeStep(const double i_dt) { m_dt = i_dt; }\
  const double getTimeStep() const { return m_dt; }\
  const double getTime() const { return m_time; }\
protected:\
  double m_dt;\
  double m_time = 0.0;\
