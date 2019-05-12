/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#include "PyCoreRobotics.hpp"
// #include <boost/python/numpy.hpp>

#include <cr/math>

//! Python bindings
void export_py_math() {
  ADD_NESTED_NAMESPACE("math")
  
  python::class_<cr::math::Conversion>("Conversion")
    .def("deg2rad", &cr::math::Conversion::deg2rad)
    .staticmethod("deg2rad")
    .def("rad2deg", &cr::math::Conversion::rad2deg)
    .staticmethod("rad2deg")
    // .def("wrapToPi", &cr::math::Conversion::wrapToPi)
    // .staticmethod("wrapToPi")
  ;
}