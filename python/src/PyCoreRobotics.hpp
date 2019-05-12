/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef PY_CORE_ROBOTICS_HPP_
#define PY_CORE_ROBOTICS_HPP_

#include <boost/python.hpp>
#include <boost/python/class.hpp>
#include <boost/python/scope.hpp>
#include <vector>
#include <string>

//! Map boost namespace
namespace python = boost::python;

//! This macro sets up the nested namespace
#define ADD_NESTED_NAMESPACE(ModuleName)\
std::string packageName = "CoreRobotics";\
python::object pyModule(python::handle<>(\
  python::borrowed(PyImport_AddModule(\
  packageName.append(ModuleName).c_str()))));\
python::scope().attr(ModuleName) = pyModule;\
python::scope moduleScope = pyModule;\

//! Export modules
void export_py_core();
void export_py_math();

#endif /* PY_CORE_ROBOTICS_HPP_ */
