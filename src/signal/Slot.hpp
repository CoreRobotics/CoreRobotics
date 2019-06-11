/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_SIGNAL_SLOT_HPP_
#define CR_SIGNAL_SLOT_HPP_

#include "Signal.hpp"
#include "core/Step.hpp"
#include <memory>
#include <mutex>

namespace cr {
namespace signal {

//------------------------------------------------------------------------------
/*!
 \class Slot
 \ingroup signal

 \brief A Slot writes data from a signal::Signal to a ReceiverType callback.
 */
//------------------------------------------------------------------------------
template <typename DataType, typename ReceiverType>
class Slot : public core::Step {

public:
  //! constructor
  Slot(std::shared_ptr<Requester<DataType>> i_signal,
       std::shared_ptr<ReceiverType> i_receiver,
       void (ReceiverType::*i_callback)(const DataType &)) {
    m_signal = i_signal;
    m_receiver = i_receiver;
    m_callback = reinterpret_cast<void (ReceiverType::*)(void *)>(i_callback);
  }

  //! destructor
  virtual ~Slot() = default;

  //! Create a new slot
  static std::shared_ptr<Slot<DataType, ReceiverType>>
  create(std::shared_ptr<Requester<DataType>> i_signal,
         std::shared_ptr<ReceiverType> i_receiver,
         void (ReceiverType::*i_callback)(const DataType &)) {
    return std::make_shared<Slot<DataType, ReceiverType>>(i_signal, i_receiver,
                                                          i_callback);
  }

  //! get the receiver
  std::shared_ptr<ReceiverType> getReceiver() { return m_receiver; }

  //! step the data push
  virtual void step() {
    void (ReceiverType::*fcn)(const DataType &) =
        reinterpret_cast<void (ReceiverType::*)(const DataType &)>(m_callback);
    DataType d = m_signal->request();
    m_mutex.lock();
    (m_receiver.get()->*fcn)(d);
    m_mutex.unlock();
  }

private:
  //! Receiver
  std::shared_ptr<ReceiverType> m_receiver;

  //! callback request functions
  void (ReceiverType::*m_callback)(void *);

  //! signal
  std::shared_ptr<Requester<DataType>> m_signal;

  //! mutex member
  std::recursive_mutex m_mutex;
};

} // namespace signal
} // namepsace cr

#endif
