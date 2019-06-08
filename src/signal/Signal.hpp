/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_SIGNAL_SIGNAL_HPP_
#define CR_SIGNAL_SIGNAL_HPP_

#include "Utility.hpp"
#include "core/Item.hpp"
#include <fstream>
#include <memory>
#include <mutex>

namespace cr {
namespace signal {

//------------------------------------------------------------------------------
/*!
 \class SignalBase
 \ingroup signal
 */
//------------------------------------------------------------------------------
class SignalBase : public core::Item {

public:
  //! constructor
  SignalBase(){};

  //! get the signal dimension
  virtual unsigned size() = 0;

  //! write signal value to ostream
  virtual void write(std::ostream &i_log) = 0;

private:
  //! type (read only)
  std::string m_type = "Signal";
};

//------------------------------------------------------------------------------
/*!
 \class Requester
 \ingroup signal
 */
//------------------------------------------------------------------------------
template <typename DataType>
class Requester : public std::enable_shared_from_this<Requester<DataType>>,
                  public SignalBase {

public:
  //! constructor
  Requester(){};

  //! request function to implement
  virtual DataType request() = 0;
};

//------------------------------------------------------------------------------
/*!
 \class Signal
 \ingroup signal

 \brief A signal writes reads from an EmitterType callback function.
 */
//------------------------------------------------------------------------------
template <typename DataType, typename EmitterType>
class Signal : public Requester<DataType> {

public:
  //! constructor
  Signal(std::shared_ptr<EmitterType> i_emitter,
         DataType (EmitterType::*i_callback)()) {
    m_emitter = i_emitter;
    m_callback = reinterpret_cast<void *(EmitterType::*)()>(i_callback);
  }

  //! Create a new signal
  static std::shared_ptr<Signal<DataType, EmitterType>>
  create(std::shared_ptr<EmitterType> i_emitter,
         DataType (EmitterType::*i_callback)()) {
    return std::make_shared<Signal<DataType, EmitterType>>(i_emitter,
                                                           i_callback);
  }

  //! get the parent
  std::shared_ptr<EmitterType> getEmitter() { return m_emitter; }

  //! request data
  virtual DataType request() {
    DataType (EmitterType::*fcn)() =
        reinterpret_cast<DataType (EmitterType::*)()>(m_callback);
    m_mutex.lock();
    DataType d = (m_emitter.get()->*fcn)();
    m_mutex.unlock();
    return d;
  }

  //! get the signal dimension
  virtual unsigned size() {
    DataType d = request();
    return Utility::size(d);
  }

  //! write signal value to ostream
  virtual void write(std::ostream &i_log) {
    DataType d = request();
    Utility::write(i_log, d);
  }

private:
  //! Parent
  std::shared_ptr<EmitterType> m_emitter;

  //! callback request functions
  void *(EmitterType::*m_callback)();

  //! mutex member
  std::recursive_mutex m_mutex;
};

} // namespace signal
} // namepsace cr

#endif
