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

#include "Utility.hpp"

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace signal {
    
    
//---------------------------------------------------------------------
/*!
 This method writes a supported signal type to log.
 
 \param[in]     i_log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//---------------------------------------------------------------------
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
    
    
//---------------------------------------------------------------------
/*!
 This method writes a string to log.
 
 \param[in]     i_log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//---------------------------------------------------------------------
void Utility::write(std::ostream& i_log, std::string i_x)
{
    i_log << i_x << ",";
}
    
    
//---------------------------------------------------------------------
/*!
 This method writes a bool to log.
 
 \param[in]     i_log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//---------------------------------------------------------------------
void Utility::write(std::ostream& i_log, bool i_x)
{
    i_log << i_x << ",";
}
    
    
//---------------------------------------------------------------------
/*!
 This method writes an int to log.
 
 \param[in]     log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//---------------------------------------------------------------------
void Utility::write(std::ostream& i_log, int i_x)
{
    i_log << i_x << ",";
}
    
    
//---------------------------------------------------------------------
/*!
 This method writes a float to log.
 
 \param[in]     log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//---------------------------------------------------------------------
void Utility::write(std::ostream& i_log, float i_x)
{
    i_log << i_x << ",";
}
    
    
//---------------------------------------------------------------------
/*!
 This method writes a double to log.
 
 \param[in]     log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//---------------------------------------------------------------------
void Utility::write(std::ostream& i_log, double i_x)
{
    i_log << i_x << ",";
}
    
    
//---------------------------------------------------------------------
/*!
 This method writes a vector to log.
 
 \param[in]     log - the log to write the variable to
 \param[in]     i_x - the value to write
 */
//---------------------------------------------------------------------
void Utility::write(std::ostream& i_log, Eigen::VectorXd i_x)
{
    for (int k = 0; k < i_x.size(); k++) {
        i_log << i_x(k) << ",";
    }
    
}
    
    
//---------------------------------------------------------------------
/*!
 This method returns the size of a supported signal type
 
 \param[in]     i_x - the value to write
 */
//---------------------------------------------------------------------
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
    


}
}
// end namespace
//---------------------------------------------------------------------
