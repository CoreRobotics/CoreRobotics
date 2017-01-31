//===========================================================================
/*
    Software License Agreement (BSD-3-Clause License)
    Copyright (c) 2017, CoreRobotics.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

    * Neither the name of CoreRobotics nor the names of its contributors
    may be used to endorse or promote products derived from this
    software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author CoreRobotics Project
    \author www.corerobotics.org
    \author Parker Owan
    \version 0.0
 
*/
//===========================================================================

#ifndef corerobotics_hpp
#define corerobotics_hpp

//===========================================================================
/*! \mainpage Introduction

The CoreRobotics libraries provide packages for real-time robot control
including kinematics, estimation, percetion, modeling and interfacing with
items in the world. The libraries are written in C++ and are intended to be
cross-platform, although they have currently only been tested on Windows 8.1
and Mac OS X platforms.  The code is distributed under the BSD 3-clause
license for flexibility of use.
 
This introduction is broken down into the following sections.
- \subpage install
- \subpage overview
- \subpage gettingstarted
- \subpage nextsteps
- \subpage references

A note on units:  All units in the library, unless specified, are in SI
(international standard), i.e.: radians, meters, kilograms, etc...
*/
//---------------------------------------------------------------------------
/*!
\page install Installation

The easiest way to utilize the CoreRobotics libraries is to simply include
the CoreRobotics.hpp header file in your code and set the header search path
to ./src/ for your compiler IDE.  Details are described below.

\section install_windows Windows Visual Studio
Todo
 
\section install_mac Mac XCode
Todo
 
\section install_codeblocks Code::Blocks
Todo

\section install_make Make
Todo
*/
//---------------------------------------------------------------------------
/*!
\page overview Overview
Todo
*/
//---------------------------------------------------------------------------
/*!
\page gettingstarted Getting Started
Todo

\section kinematics Kinematics
Todo
*/
//---------------------------------------------------------------------------
/*!
\page nextsteps Next Steps
Todo
*/
//---------------------------------------------------------------------------
/*!
\page references References
Todo 
*/
//===========================================================================

//===========================================================================
/*!
\file CoreRobotics.hpp
\brief CoreRobotics header.
//! \details Todo
*/
//===========================================================================


//---------------------------------------------------------------------------
//! \defgroup core Core
//! \brief Implements core functionality.
//! \details Todo
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
//! \defgroup math Math
//! \brief Implements math components.
//! \details Todo
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
//! \defgroup physics Physics
//! \brief Implements physics (kinematics and dynamics) representations.
//! \details Todo
//---------------------------------------------------------------------------
#include "../src/physics/CRFrame.hpp"
#include "../src/physics/CRFrameEuler.hpp"
#include "../src/physics/CRFrameDH.hpp"


//---------------------------------------------------------------------------
//! \defgroup models Models
//! \brief Implements model (sensor, motion, and noise) representations.
//! \details Todo
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
//! \defgroup world World
//! \brief Implements the world for interacting with multiple objects.
//! \details Todo
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
//! \defgroup estimators Estimators
//! \brief Implements estimators for recovering model states.
//! \details Todo
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
//! \defgroup controllers Controllers
//! \brief Implements controllers for regulating model motion.
//! \details Todo
//---------------------------------------------------------------------------


#endif
