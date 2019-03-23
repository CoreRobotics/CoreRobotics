/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "signal/Utility.hpp"

namespace cr {
namespace signal {

//------------------------------------------------------------------------------
/*!
 This method writes a supported signal type to log.

 \param[in]     i_log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//------------------------------------------------------------------------------
/*
void Utility::write(std::ostream& i_log, SupportedSignalTypes i_x)
{
if (i_x.which() == TYPE_STRING)
{
    Utility::write(i_log, boost::get<std::string>(i_x));
}
else if (i_x.which() == TYPE_BOOL)
{
    Utility::write(i_log, boost::get<bool>(i_x));
}
else if (i_x.which() == TYPE_INT)
{
    Utility::write(i_log, boost::get<int>(i_x));
}
else if (i_x.which() == TYPE_FLOAT)
{
    Utility::write(i_log, boost::get<float>(i_x));
}
else if (i_x.which() == TYPE_DOUBLE)
{
    Utility::write(i_log, boost::get<double>(i_x));
}
else if (i_x.which() == TYPE_VECTOR)
{
    Utility::write(i_log, boost::get<Eigen::VectorXd>(i_x));
}
}
*/

//------------------------------------------------------------------------------
/*!
 This method writes a string to log.

 \param[in]     i_log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//------------------------------------------------------------------------------
void Utility::write(std::ostream &i_log, std::string i_x) {
  i_log << i_x << ",";
}

//------------------------------------------------------------------------------
/*!
 This method writes a bool to log.

 \param[in]     i_log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//------------------------------------------------------------------------------
void Utility::write(std::ostream &i_log, bool i_x) { i_log << i_x << ","; }

//------------------------------------------------------------------------------
/*!
 This method writes an int to log.

 \param[in]     log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//------------------------------------------------------------------------------
void Utility::write(std::ostream &i_log, int i_x) { i_log << i_x << ","; }

//------------------------------------------------------------------------------
/*!
 This method writes a float to log.

 \param[in]     log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//------------------------------------------------------------------------------
void Utility::write(std::ostream &i_log, float i_x) { i_log << i_x << ","; }

//------------------------------------------------------------------------------
/*!
 This method writes a double to log.

 \param[in]     log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//------------------------------------------------------------------------------
void Utility::write(std::ostream &i_log, double i_x) { i_log << i_x << ","; }

//------------------------------------------------------------------------------
/*!
 This method writes a vector to log.

 \param[in]     log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//------------------------------------------------------------------------------
void Utility::write(std::ostream &i_log, Eigen::VectorXd i_x) {
  for (int k = 0; k < i_x.size(); k++) {
    i_log << i_x(k) << ",";
  }
}

//------------------------------------------------------------------------------
/*!
 This method returns the size of a supported signal type

 \param[in]     i_x - the value to write
 */
//------------------------------------------------------------------------------
/*
unsigned Utility::size(SupportedSignalTypes i_x)
{
if (x.which() == TYPE_STRING)
{
    return Utility::size(boost::get<std::string>(i_x));
}
else if (x.which() == TYPE_BOOL)
{
    return Utility::size(boost::get<bool>(i_x));
}
else if (x.which() == TYPE_INT)
{
    return Utility::size(boost::get<int>(i_x));
}
else if (x.which() == TYPE_FLOAT)
{
    return Utility::size(boost::get<float>(i_x));
}
else if (x.which() == TYPE_DOUBLE)
{
    return Utility::size(boost::get<double>(i_x));
}
else if (x.which() == TYPE_VECTOR)
{
    return Utility::size(boost::get<Eigen::VectorXd>(i_x));
}
return 0;
}
 */

} // namespace signal
} // namepsace cr
