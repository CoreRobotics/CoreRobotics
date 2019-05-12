/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#include "PyCoreRobotics.hpp"

BOOST_PYTHON_MODULE(CoreRobotics) {
  // specify that this module is actually a package
  python::object package = python::scope();
  package.attr("__path__") = "CoreRobotics";

  export_py_core();
  export_py_math();
}