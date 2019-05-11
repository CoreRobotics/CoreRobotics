/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

//! Classes that have a read only parameter aspect should include this macro
#define CR_ASPECT_PARAMETER_READ(ParamType)\
public:\
  const ParamType& getParameters() { return m_param; }\
private:\
  ParamType m_param;\

//! Classes that have a read/write parameter aspect should include this macro
#define CR_ASPECT_PARAMETER_WRITE(ParamType)\
CR_ASPECT_PARAMETER_READ(ParamType)\
public:\
  void setParameters(const ParamType& p) { m_param = p; }\

//! Classes that have a mutable parameter aspect should include this macro
#define CR_ASPECT_PARAMETER_MUTABLE(ParamType)\
CR_ASPECT_PARAMETER_WRITE(ParamType)\
public:\
  ParamType* parameters() { return &m_param; }\
