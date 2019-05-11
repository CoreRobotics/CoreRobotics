/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

//! Classes that have a read only state aspect should include this macro
#define CR_ASPECT_STATE_READ(StateType)\
public:\
  const StateType& getState() { return m_state; }\
private:\
  StateType m_state;\

//! Classes that have a read/write state aspect should include this macro
#define CR_ASPECT_STATE_WRITE(StateType)\
CR_ASPECT_STATE_READ(StateType)\
public:\
  void setState(const StateType& i_x) { m_state = i_x; }\

//! Classes that have a mutable state aspect should include this macro
#define CR_ASPECT_STATE_MUTABLE(StateType)\
CR_ASPECT_STATE_WRITE(StateType)\
public:\
  StateType* state() { return &m_state; }\