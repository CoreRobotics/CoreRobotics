/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#include "PyCoreRobotics.hpp"
// #include <boost/python/numpy.hpp>

#include <cr/math>

// double          (cr::math::Conversion::*wrapToPi1)(const double)           = &cr::math::Conversion::wrapToPi;
// Eigen::VectorXd (cr::math::Conversion::*wrapToPi2)(const Eigen::VectorXd&) = &cr::math::Conversion::wrapToPi;

//! Python bindings
void export_py_math() {
  ADD_NESTED_NAMESPACE("math")
  
  python::class_<cr::math::Conversion>("Conversion")
    .def("deg2rad", &cr::math::Conversion::deg2rad)
    .staticmethod("deg2rad")
    .def("rad2deg", &cr::math::Conversion::rad2deg)
    .staticmethod("rad2deg")
    //.def("wrapToPi", wrapToPi1)
    //.staticmethod("wrapToPi")
    //.def("wrapToPi", wrapToPi2)
    //.staticmethod("wrapToPi")
  ;
}