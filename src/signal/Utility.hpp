//=====================================================================
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
 
 \project CoreRobotics Project
 \url     www.corerobotics.org
 \author  Parker Owan
 
 */
//---------------------------------------------------------------------
// Begin header definition

#ifndef CR_UTILITY_HPP_
#define CR_UTILITY_HPP_


#include "Eigen/Dense"
#include <iostream>
#include <string>
// #include <boost/variant.hpp>


//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace signal {
    
    
//! Supported signal types list
// typedef boost::variant<std::string, bool, int, float, double, Eigen::VectorXd> SupportedSignalTypes;
    
    
//! Enumerator for signal types
enum SignalType {
    TYPE_STRING,
    TYPE_BOOL,
    TYPE_INT,
    TYPE_FLOAT,
    TYPE_DOUBLE,
    TYPE_VECTOR,
};
    
    

//---------------------------------------------------------------------
/*!
 \class Utility
 \ingroup signal
 
 \brief This static class provides utilities for supported signal types
 
 \details
 ## Description
 
 */
//---------------------------------------------------------------------
class Utility
{
    
    
    // Static methods for writing data
    public:
    

        //! write a supported signal type
        // static void write(std::ostream& i_log, SupportedSignalTypes i_x);
    
        //! write string
        static void write(std::ostream& i_log, std::string i_x);
    
        //! write bool
        static void write(std::ostream& i_log, bool i_x);
    
        //! write int
        static void write(std::ostream& i_log, int i_x);
    
        //! write float
        static void write(std::ostream& i_log, float i_x);
    
        //! write double
        static void write(std::ostream& i_log, double i_x);
    
        //! write vector
        static void write(std::ostream& i_log, Eigen::VectorXd i_x);
    
    
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


}
}
// end namespace
//---------------------------------------------------------------------


#endif

