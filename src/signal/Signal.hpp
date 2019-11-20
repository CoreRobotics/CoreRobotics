/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_SIGNAL_SIGNAL_HPP_
#define CR_SIGNAL_SIGNAL_HPP_

#include "core/Item.hpp"
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>

namespace cr {
namespace signal {

//------------------------------------------------------------------------------
/*!
 \class Requester
 \ingroup signal
 */
//------------------------------------------------------------------------------
template <typename DataType>
class Requester : public std::enable_shared_from_this<Requester<DataType>> {

public:
  //! constructor
  Requester() = default;

  //! destructor
  virtual ~Requester() = default;

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
class Signal : public Requester<DataType>, public core::Item {

public:
  //! constructor
  Signal(std::shared_ptr<EmitterType> i_emitter,
         DataType (EmitterType::*i_callback)()) {
    m_emitter = i_emitter;
    m_callback = reinterpret_cast<void *(EmitterType::*)()>(i_callback);
  }

  //! destructor
  virtual ~Signal() = default;

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

private:
  //! Parent
  std::shared_ptr<EmitterType> m_emitter;

  //! callback request functions
  void *(EmitterType::*m_callback)();

  //! mutex member
  std::recursive_mutex m_mutex;
};

template <typename DataType>
class MySignal : public cr::signal::Requester<DataType> {
public:
  MySignal(std::function<DataType()> i_callback) { m_callback = i_callback; }

  //! request data
  virtual DataType request() {
    m_data = m_callback();
    return m_data;
  }

private:
  std::function<DataType()> m_callback;
  DataType m_data;
};

template <typename DataType> class MySlot {
public:
  MySlot(std::function<void(DataType)> i_callback) { m_callback = i_callback; }

  //! request data
  virtual void step() {
    DataType d;
    m_callback(d);
  }

private:
  std::function<void(DataType)> m_callback;
};

} // namespace signal
} // namespace cr

#endif
