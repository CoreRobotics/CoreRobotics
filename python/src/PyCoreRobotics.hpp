/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef PY_CORE_ROBOTICS_HPP_
#define PY_CORE_ROBOTICS_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>

//! Map pybind11 namespace
namespace py = pybind11;

//! Export modules
void export_py_core(py::module& m);
void export_py_runtime(py::module& m);
void export_py_math(py::module& m);

#endif /* PY_CORE_ROBOTICS_HPP_ */
