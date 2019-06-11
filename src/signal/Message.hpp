/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_SIGNAL_MESSAGE_HPP_
#define CR_SIGNAL_MESSAGE_HPP_

#include "Signal.hpp"
#include "core/Step.hpp"
#include "core/Item.hpp"
#include "Eigen/Dense"
#include <iostream>
#include <string>

namespace cr {
namespace signal {

//! Enumerator for generic message types
enum GenericTypes {
  TYPE_STRING,
  TYPE_BOOL,
  TYPE_INT,
  TYPE_FLOAT,
  TYPE_DOUBLE,
  TYPE_VECTOR,
};

//------------------------------------------------------------------------------
/*!
 \class MessageBase
 \ingroup signal
 */
//------------------------------------------------------------------------------
class MessageBase
  : public std::enable_shared_from_this<MessageBase>, public core::Item {

public:
  //! constructor
  MessageBase() = default;

  //! destructor
  virtual ~MessageBase() = default;

  //! get the signal dimension
  virtual unsigned size() = 0;

  //! write signal value to ostream
  virtual void write(std::ostream &i_log) = 0;

private:
  //! type (read only)
  std::string m_type = "Message";
};


//------------------------------------------------------------------------------
/*!
 \class GenericSerializer
 \ingroup signal

 \brief This class provides serialization for common message types.
 */
//------------------------------------------------------------------------------
class GenericSerializer {

public:

  //! write string
  static void write(std::ostream &i_log, std::string i_x);

  //! write bool
  static void write(std::ostream &i_log, bool i_x);

  //! write int
  static void write(std::ostream &i_log, int i_x);

  //! write float
  static void write(std::ostream &i_log, float i_x);

  //! write double
  static void write(std::ostream &i_log, double i_x);

  //! write vector
  static void write(std::ostream &i_log, Eigen::VectorXd i_x);

public:

  //! size string
  static unsigned size(std::string i_x) { return 1; }

  //! size bool
  static unsigned size(bool i_x) { return 1; }

  //! size int
  static unsigned size(int i_x) { return 1; }

  //! size float
  static unsigned size(float i_x) { return 1; }

  //! size double
  static unsigned size(double i_x) { return 1; }

  //! size vector
  static unsigned size(Eigen::VectorXd i_x) { return i_x.size(); }
};


//------------------------------------------------------------------------------
/*!
 \class Tap
 \ingroup signal

 \brief A message that supports serialization from a signal.
 */
//------------------------------------------------------------------------------
template <typename DataType, typename Serializer = GenericSerializer>
class Tap : public MessageBase {

public:
  //! constructor
  Tap(std::shared_ptr<Requester<DataType>> i_requester) 
    : MessageBase(), m_requester(i_requester) {}

  //! destructor
  virtual ~Tap() = default;

  //! Create a new generic message
  static std::shared_ptr<Tap<DataType, Serializer>>
  create(std::shared_ptr<Requester<DataType>> i_requester) {
    return std::make_shared<Tap<DataType, Serializer>>(i_requester);
  }

  //! get the signal dimension
  virtual unsigned size() {
    return Serializer::size(m_requester->request());
  }

  //! write signal value to ostream
  virtual void write(std::ostream &i_log) {
    Serializer::write(i_log, m_requester->request());
  }

private:
  //! signal pointer
  std::shared_ptr<Requester<DataType>> m_requester;
};


} // namespace signal
} // namepsace cr

#endif
