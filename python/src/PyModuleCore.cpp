/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#include "PyCoreRobotics.hpp"
// #include <boost/python/numpy.hpp>
#include <boost/python/wrapper.hpp>
#include <boost/python/call.hpp>

#include <cr/core>

//! See: https://www.boost.org/doc/libs/1_50_0/libs/python/doc/v2/wrapper.html
class StepWrapper
  : public cr::core::Step, public python::wrapper<cr::core::Step>
{
public:
  void step() {
    if (python::override step = this->get_override("step")) {
      step();
      return;
    } else {
      cr::core::Step::step();
      return;
    }
  }
  void onStart() {
    if (python::override onStart = this->get_override("onStart")) {
      onStart();
      return;
    } else {
      cr::core::Step::onStart();
      return;
    }
  }
  void onStop() {
    if (python::override onStop = this->get_override("onStop")) {
      onStop();
      return;
    } else {
      cr::core::Step::onStop();
      return;
    }
  }
  void default_step() { return this->Step::step(); }
  void default_onStart() { return this->Step::onStart(); }
  void default_onStop() { return this->Step::onStop(); }
};

//! Python bindings
void export_py_core () {
  ADD_NESTED_NAMESPACE("core")
  
  python::enum_<cr::core::Result>("Result")
    .value("CR_RESULT_SUCCESS", cr::core::CR_RESULT_SUCCESS)
    .value("CR_RESULT_SINGULAR", cr::core::CR_RESULT_SINGULAR)
    .value("CR_RESULT_UNWRITABLE", cr::core::CR_RESULT_UNWRITABLE)
    .value("CR_RESULT_BAD_IC", cr::core::CR_RESULT_BAD_IC)
    .value("CR_RESULT_NOT_FOUND", cr::core::CR_RESULT_NOT_FOUND)
    .export_values()
    ;

  python::enum_<cr::core::RunState>("RunState")
    .value("CR_RUN_STATE_RUNNING", cr::core::CR_RUN_STATE_RUNNING)
    .value("CR_RUN_STATE_STOPPED", cr::core::CR_RUN_STATE_STOPPED)
    .value("CR_RUN_STATE_PAUSED", cr::core::CR_RUN_STATE_PAUSED)
    .export_values()
    ;

  python::enum_<cr::core::ThreadPriority>("ThreadPriority")
    .value("CR_PRIORITY_LOWEST", cr::core::CR_PRIORITY_LOWEST)
    .value("CR_PRIORITY_LOW", cr::core::CR_PRIORITY_LOW)
    .value("CR_PRIORITY_NORMAL", cr::core::CR_PRIORITY_NORMAL)
    .value("CR_PRIORITY_HIGH", cr::core::CR_PRIORITY_HIGH)
    .value("CR_PRIORITY_HIGHEST", cr::core::CR_PRIORITY_HIGHEST)
    .export_values()
    ;

  python::class_<cr::core::Clock>("Clock")
    .def("startTimer", &cr::core::Clock::startTimer)
    .def("getElapsedTime", &cr::core::Clock::getElapsedTime)
    .def("sleep", &cr::core::Clock::sleep)
  ;

  python::class_<cr::core::Item>("Item")
    .def("setName", &cr::core::Item::setName)
    .def("getName", &cr::core::Item::getName)
    .def("setIcon", &cr::core::Item::setIcon)
    .def("getIcon", &cr::core::Item::getIcon)
    .def("getType", &cr::core::Item::getType)
  ;

  python::class_<cr::core::Loop, cr::core::LoopPtr>("Loop")
    .def(python::init<double>())
    .def("__init__", python::make_constructor(&cr::core::StepList::create))
    .def("create", &cr::core::Loop::create)
    .staticmethod("create")
    .def("start", &cr::core::Loop::start)
    .def("pause", &cr::core::Loop::pause)
    .def("stop", &cr::core::Loop::stop)
    .def("attach", &cr::core::Loop::attach)
    .def("setPriority", &cr::core::Loop::setPriority)
    .def("setUpdateRate", &cr::core::Loop::setUpdateRate)
    .def("getUpdateRate", &cr::core::Loop::getUpdateRate)
    .def("getCurrentTime", &cr::core::Loop::getCurrentTime)
  ;

  python::class_<StepWrapper, boost::noncopyable, std::shared_ptr<StepWrapper>>("Step")
    .def("step", &cr::core::Step::step, &StepWrapper::default_step)
    .def("onStart", &cr::core::Step::onStart, &StepWrapper::default_onStart)
    .def("onStop", &cr::core::Step::onStop, &StepWrapper::default_onStop)
  ;

  python::class_<cr::core::StepList, cr::core::StepListPtr,
      python::bases<cr::core::Step>>("StepList")
    .def("__init__", python::make_constructor(&cr::core::StepList::create))
    .def("create", &cr::core::StepList::create)
    .staticmethod("create")
    .def("attach", &cr::core::StepList::attach)
    .def("step", &cr::core::StepList::step)
  ;
}