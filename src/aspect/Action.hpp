/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

//! Classes that have a read only action aspect should include this macro
#define CR_ASPECT_ACTION_READ(ActionType)\
public:\
  const ActionType& getAction() const { return m_action; }\
protected:\
  ActionType m_action;\

//! Classes that have a read/write action aspect should include this macro
#define CR_ASPECT_ACTION_WRITE(ActionType)\
CR_ASPECT_ACTION_READ(ActionType)\
public:\
  void setAction(const ActionType& i_x) { m_action = i_x; }\

//! Classes that have a mutable action aspect should include this macro
#define CR_ASPECT_ACTION_MUTABLE(ActionType)\
CR_ASPECT_ACTION_WRITE(ActionType)\
public:\
  ActionType* action() { return &m_action; }\
