/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "Message.hpp"

namespace cr {
namespace signal {

//------------------------------------------------------------------------------
/*!
 This method writes a string to log.

 \param[in]     i_log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//------------------------------------------------------------------------------
void GenericSerializer::write(std::ostream &i_log, std::string i_x) {
  i_log << i_x << ",";
}

//------------------------------------------------------------------------------
/*!
 This method writes a bool to log.

 \param[in]     i_log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//------------------------------------------------------------------------------
void GenericSerializer::write(std::ostream &i_log, bool i_x) {
  i_log << i_x << ",";
}

//------------------------------------------------------------------------------
/*!
 This method writes an int to log.

 \param[in]     log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//------------------------------------------------------------------------------
void GenericSerializer::write(std::ostream &i_log, int i_x) {
  i_log << i_x << ",";
}

//------------------------------------------------------------------------------
/*!
 This method writes a float to log.

 \param[in]     log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//------------------------------------------------------------------------------
void GenericSerializer::write(std::ostream &i_log, float i_x) {
  i_log << i_x << ",";
}

//------------------------------------------------------------------------------
/*!
 This method writes a double to log.

 \param[in]     log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//------------------------------------------------------------------------------
void GenericSerializer::write(std::ostream &i_log, double i_x) {
  i_log << i_x << ",";
}

//------------------------------------------------------------------------------
/*!
 This method writes a vector to log.

 \param[in]     log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//------------------------------------------------------------------------------
void GenericSerializer::write(std::ostream &i_log, Eigen::VectorXd i_x) {
  for (int k = 0; k < i_x.size(); k++) {
    i_log << i_x(k) << ",";
  }
}

} // namespace signal
} // namespace cr
