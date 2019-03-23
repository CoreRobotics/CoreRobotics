/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_UTILITY_HPP_
#define CR_UTILITY_HPP_

#include "Eigen/Dense"
#include <iostream>
#include <string>
// #include <boost/variant.hpp>

namespace cr {
namespace signal {

//! Supported signal types list
// typedef boost::variant<std::string, bool, int, float, double,
// Eigen::VectorXd> SupportedSignalTypes;

//! Enumerator for signal types
enum SignalType {
  TYPE_STRING,
  TYPE_BOOL,
  TYPE_INT,
  TYPE_FLOAT,
  TYPE_DOUBLE,
  TYPE_VECTOR,
};

//------------------------------------------------------------------------------
/*!
 \class Utility
 \ingroup signal

 \brief This static class provides utilities for supported signal types

 \details
 ## Description

 */
//------------------------------------------------------------------------------
class Utility {

  // Static methods for writing data
public:
  //! write a supported signal type
  // static void write(std::ostream& i_log, SupportedSignalTypes i_x);

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

  // Static methods for getting data size
public:
  //! size of the supported signal types
  // static unsigned size(SupportedSignalTypes i_x);

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

} // namespace signal
} // namepsace cr

#endif
