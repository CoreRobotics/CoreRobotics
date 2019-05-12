/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

//! Classes that have a read only parameter aspect should include this macro
#define CR_ASPECT_PARAMETER_READ(ParameterType)\
public:\
  const ParameterType& getParameters() const { return m_parameters; }\
protected:\
  ParameterType m_parameters;\

//! Classes that have a read/write parameter aspect should include this macro
#define CR_ASPECT_PARAMETER_WRITE(ParameterType)\
CR_ASPECT_PARAMETER_READ(ParameterType)\
public:\
  void setParameters(const ParameterType& i_p) { m_parameters = i_p; }\

//! Classes that have a mutable parameter aspect should include this macro
#define CR_ASPECT_PARAMETER_MUTABLE(ParameterType)\
CR_ASPECT_PARAMETER_WRITE(ParameterType)\
public:\
  ParameterType* parameters() { return &m_parameters; }\
