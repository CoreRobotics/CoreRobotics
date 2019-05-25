/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#include "PyCoreRobotics.hpp"

#include <cr/core>

//! Step trampoline class
class PyStep : public cr::core::Step {
public:
    // Inherit the constructors
    using Step::Step;

    // Trampoline (need one for each virtual function)
    void step() override {
      PYBIND11_OVERLOAD(
        void,
        cr::core::Step,
        step);
    }

    // Trampoline (need one for each virtual function)
    void onStart() override {
      PYBIND11_OVERLOAD(
        void,
        cr::core::Step,
        onStart);
    }

    // Trampoline (need one for each virtual function)
    void onStop() override {
      PYBIND11_OVERLOAD(
        void,
        cr::core::Step,
        onStop);
    }
};

void export_py_core(py::module& m) {

  py::enum_<cr::core::Result>(m,  "Result")
    .value("CR_RESULT_SUCCESS", cr::core::CR_RESULT_SUCCESS)
    .value("CR_RESULT_SINGULAR", cr::core::CR_RESULT_SINGULAR)
    .value("CR_RESULT_UNWRITABLE", cr::core::CR_RESULT_UNWRITABLE)
    .value("CR_RESULT_BAD_IC", cr::core::CR_RESULT_BAD_IC)
    .value("CR_RESULT_NOT_FOUND", cr::core::CR_RESULT_NOT_FOUND)
    .export_values()
    ;

  py::class_<cr::core::Clock>(m, "Clock")
    .def(py::init())
    .def("startTimer", &cr::core::Clock::startTimer)
    .def("getElapsedTime", &cr::core::Clock::getElapsedTime)
    .def("sleep", &cr::core::Clock::sleep)
  ;

  py::class_<cr::core::Item>(m, "Item")
    .def(py::init())
    .def("setName", &cr::core::Item::setName)
    .def("getName", &cr::core::Item::getName)
    .def("setIcon", &cr::core::Item::setIcon)
    .def("getIcon", &cr::core::Item::getIcon)
    .def("getType", &cr::core::Item::getType)
  ;

  py::class_<cr::core::Step, cr::core::StepPtr, PyStep>(m,  "Step")
    .def(py::init<>())
    .def_static("create", &cr::core::Step::create)
    .def("step", &cr::core::Step::step)
    .def("onStart", &cr::core::Step::onStart)
    .def("onStop", &cr::core::Step::onStop)
    .def("ptr", &cr::core::Step::ptr)
  ;

  py::class_<cr::core::StepList, cr::core::StepListPtr,
      cr::core::Step>(m, "StepList")
    .def("attach", &cr::core::StepList::attach)
    .def("step", &cr::core::StepList::step)
  ;
}
