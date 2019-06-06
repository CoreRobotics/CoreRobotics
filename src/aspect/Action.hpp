/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef CR_ASPECT_ACTION_HPP_
#define CR_ASPECT_ACTION_HPP_

/*!
  \def CR_ASPECT_ACTION_READ(ActionType)
  \ingroup aspect
  Classes that have a read only action aspect should include this macro.

  \brief This macro adds class members.
  \param ActionType  The action data type.
*/
#define CR_ASPECT_ACTION_READ(ActionType) \
public: \
  const ActionType& getAction() const { return m_action; } \
protected: \
  ActionType m_action;

/*!
  \def CR_ASPECT_ACTION_WRITE(ActionType)
  \ingroup aspect
  Classes that have a read/write action aspect should include this macro

  \brief This macro adds class members.
  \param ActionType  The action data type.
*/
#define CR_ASPECT_ACTION_WRITE(ActionType) \
CR_ASPECT_ACTION_READ(ActionType) \
public: \
  void setAction(const ActionType& i_x) { m_action = i_x; }

/*!
  \def CR_ASPECT_ACTION_MUTABLE(ActionType)
  \ingroup aspect
  Classes that have a mutable action aspect should include this macro

  \brief This macro adds class members.
  \param ActionType  The action data type.
*/
#define CR_ASPECT_ACTION_MUTABLE(ActionType) \
CR_ASPECT_ACTION_WRITE(ActionType) \
public: \
  ActionType* action() { return &m_action; }

#endif
