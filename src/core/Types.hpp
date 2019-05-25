/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef CR_TYPES_HPP_
#define CR_TYPES_HPP_

#include <string>

namespace cr {
namespace core {

//---------------------------------------------------------------------
/*!
 \file Types.hpp
 \brief Enumerator and basic types for common usage across all modules.
 */
//---------------------------------------------------------------------

//! Result enumerator for consistent operation result flags.
enum Result {
  CR_RESULT_SUCCESS,
  CR_RESULT_SINGULAR,
  CR_RESULT_UNWRITABLE,
  CR_RESULT_BAD_IC,
  CR_RESULT_NOT_FOUND,
};

} // namepsace core
} // namespace cr

#endif
