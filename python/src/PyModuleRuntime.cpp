/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#include "PyCoreRobotics.hpp"

#include <cr/runtime>

//! Loop trampoline class
class PyLoop : public cr::runtime::Loop {
public:
    // Inherit the constructors
    using Loop::Loop;

    // Trampoline (need one for each virtual function)
    void start() override {
      py::gil_scoped_acquire acquire;

      PYBIND11_OVERLOAD(
        void,
        cr::runtime::Loop,
        start);
    }
};

void export_py_runtime(py::module& m) {

  py::enum_<cr::runtime::RunState>(m, "RunState")
    .value("CR_RUN_STATE_RUNNING", cr::runtime::CR_RUN_STATE_RUNNING)
    .value("CR_RUN_STATE_STOPPED", cr::runtime::CR_RUN_STATE_STOPPED)
    .value("CR_RUN_STATE_PAUSED", cr::runtime::CR_RUN_STATE_PAUSED)
    .export_values()
    ;

  py::enum_<cr::runtime::ThreadPriority>(m, "ThreadPriority")
    .value("CR_PRIORITY_LOWEST", cr::runtime::CR_PRIORITY_LOWEST)
    .value("CR_PRIORITY_LOW", cr::runtime::CR_PRIORITY_LOW)
    .value("CR_PRIORITY_NORMAL", cr::runtime::CR_PRIORITY_NORMAL)
    .value("CR_PRIORITY_HIGH", cr::runtime::CR_PRIORITY_HIGH)
    .value("CR_PRIORITY_HIGHEST", cr::runtime::CR_PRIORITY_HIGHEST)
    .export_values()
    ;

  py::class_<cr::runtime::Loop, cr::runtime::LoopPtr, PyLoop>(m, "Loop")
    .def(py::init<double>())
    .def_static("create", &cr::runtime::Loop::create)
    .def("start", &cr::runtime::Loop::start, py::call_guard<py::gil_scoped_release>())
    .def("pause", &cr::runtime::Loop::pause)
    .def("stop", &cr::runtime::Loop::stop)
    .def("attach", &cr::runtime::Loop::attach)
    .def("setPriority", &cr::runtime::Loop::setPriority)
    .def("getPriority", &cr::runtime::Loop::getPriority)
    .def("setUpdateRate", &cr::runtime::Loop::setUpdateRate)
    .def("getUpdateRate", &cr::runtime::Loop::getUpdateRate)
    .def("getCurrentTime", &cr::runtime::Loop::getCurrentTime)
  ;
}
