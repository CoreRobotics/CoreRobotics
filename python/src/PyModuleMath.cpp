/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#include "PyCoreRobotics.hpp"

#include <cr/math>

// Static overload workaround
double py_wrapToPiDouble(const double angle) {
  return cr::math::Conversion::wrapToPi(angle);
}

// Static overload workaround
Eigen::VectorXd py_wrapToPiEigen(const Eigen::VectorXd angle) {
  return cr::math::Conversion::wrapToPi(angle);
}

// Static overload workaround
Eigen::VectorXd py_forwardEulerStep(
    std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)>
        system,
    double t, Eigen::VectorXd x, Eigen::VectorXd u, double dt) {
  return cr::math::Integration::forwardEulerStep(system, t, x, u, dt);
}

// Static overload workaround
Eigen::VectorXd py_rungeKuttaStep(
    std::function<Eigen::VectorXd(double, Eigen::VectorXd, Eigen::VectorXd)>
        system,
    double t, Eigen::VectorXd x, Eigen::VectorXd u, double dt) {
  return cr::math::Integration::rungeKuttaStep(system, t, x, u, dt);
}

void export_py_math(py::module &m) {

  py::class_<cr::math::Conversion>(m, "Conversion")
      .def_static("deg2rad", &cr::math::Conversion::deg2rad)
      .def_static("rad2deg", &cr::math::Conversion::rad2deg)
      .def_static("wrapToPi", (double (*)(const double)) & py_wrapToPiDouble)
      .def_static("wrapToPi", (Eigen::VectorXd(*)(const Eigen::VectorXd &)) &
                                  py_wrapToPiEigen);

  py::class_<cr::math::Integration>(m, "Integration")
      .def_static("forwardEulerStep", &py_forwardEulerStep)
      .def_static("rungeKuttaStep", &py_rungeKuttaStep);
}
