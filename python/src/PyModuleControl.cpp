/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#include "PyCoreRobotics.hpp"

#include <cr/control>

void export_py_control(py::module& m) {

  py::class_<cr::control::Waypoint>(m, "Waypoint")
    .def(py::init<double>())
    .def_readwrite("time", &cr::control::Waypoint::time)
    .def_readwrite("position", &cr::control::Waypoint::position)
    .def_readwrite("velocity", &cr::control::Waypoint::velocity)
    .def_readwrite("acceleration", &cr::control::Waypoint::acceleration)
    .def_readwrite("jerk", &cr::control::Waypoint::jerk)
  ;

/*
  py::class_<cr::control::MinimumJerk> mjtg(m, "MinimumJerk");
  mjtg
    .def(py::init<cr::control::MinimumJerk::Parameters, cr::control::Waypoint>())
    .def_static("create", &cr::control::MinimumJerk::create)
    .def("setParameters", &cr::control::MinimumJerk::setParameters)
    .def("getParameters", &cr::control::MinimumJerk::getParameters)
    .def("parameter", &cr::control::MinimumJerk::parameter)
    .def("getAction", &cr::control::MinimumJerk::getAction)
    .def("setTime", &cr::control::MinimumJerk::getTime)
    .def("setTime", &cr::control::MinimumJerk::getTime)
    .def("policyCallback", &cr::control::MinimumJerk::policyCallback)
  ;
*/

  py::class_<cr::control::MinimumJerk::Parameters>(m, "MinimumJerkParameters")
    .def(py::init<std::size_t>())
    .def("coefficients", &cr::control::MinimumJerk::Parameters::coefficients)
    .def("getDuration", &cr::control::MinimumJerk::Parameters::getDuration)
    .def("solve", (cr::core::Result (cr::control::MinimumJerk::Parameters::*)(
      const cr::control::Waypoint&, const cr::control::Waypoint&)) 
      &cr::control::MinimumJerk::Parameters::solve)
    .def("solve", (cr::core::Result (cr::control::MinimumJerk::Parameters::*)(
      const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
      const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
      double)) 
      &cr::control::MinimumJerk::Parameters::solve)
  ;
}
