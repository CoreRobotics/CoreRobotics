/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef CR_ASPECT_STATE_HPP_
#define CR_ASPECT_STATE_HPP_

/*!
  \def CR_ASPECT_STATE_READ(StateType)
  \ingroup aspect
  Classes that have a read only state aspect should include this macro.

  \brief This macro adds class members.
  \param StateType  The state data type.
*/
#define CR_ASPECT_STATE_READ(StateType) \
public: \
  const StateType& getState() const { return m_state; } \
protected: \
  StateType m_state;

/*!
  \def CR_ASPECT_STATE_WRITE(StateType)
  \ingroup aspect
  Classes that have a read/write state aspect should include this macro.

  \brief This macro adds class members.
  \param StateType  The state data type.
*/
#define CR_ASPECT_STATE_WRITE(StateType) \
CR_ASPECT_STATE_READ(StateType) \
public: \
  void setState(const StateType& i_x) { m_state = i_x; }

/*!
  \def CR_ASPECT_STATE_MUTABLE(StateType)
  \ingroup aspect
  Classes that have a mutable state aspect should include this macro.

  \brief This macro adds class members.
  \param StateType  The state data type.
*/
#define CR_ASPECT_STATE_MUTABLE(StateType) \
CR_ASPECT_STATE_WRITE(StateType) \
public: \
  StateType* state() { return &m_state; }

#endif
