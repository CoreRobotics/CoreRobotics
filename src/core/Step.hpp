/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef CR_STEP_HPP_
#define CR_STEP_HPP_

#include <memory>

namespace cr {
namespace core {

//! Step shared pointer
class Step;
typedef std::shared_ptr<Step> StepPtr;

//---------------------------------------------------------------------
/*!
 \class Step
 \ingroup core

 \brief This abstract class defines callback functions that are used
 in the cr::core::Loop class.

 \details
 This abstract class defines the base callback methods needed to derive
 a call for a Loop class:

 - Step::step() is called on each iteration of core::Loop while the
 thread is running.  This function must be implemented in derived classes
 - Step::onStart() is called on the start of core::Loop.  It is optional.
 - Step::onStop() is called on the stop of core::Loop.  It is optional.
 */
//---------------------------------------------------------------------
class Step : public std::enable_shared_from_this<Step> {

//! Constructor and destructor
public:
  //! constructor
  Step() {}

  //! destructor
  virtual ~Step() {}

//! Functions to be implemented
public:
  //! The onStep function must be implemented in derived classes
  virtual void step() = 0;

  //! The onStart function can be implemented in derived classes
  virtual void onStart() {};

  //! The onStop function can be implemented in derived classes
  virtual void onStop() {};
};

}  // namespace core
}  // namepsace cr

#endif
