/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#include "PyCoreRobotics.hpp"

PYBIND11_PLUGIN(CoreRobotics) {
  py::module m("CoreRobotics", "CoreRobotics Python bindings");

  export_py_core(m);
  export_py_runtime(m);
  export_py_math(m);
  export_py_control(m);
  return m.ptr();
}
