//=====================================================================
/*
 Software License Agreement (BSD-3-Clause License)
 Copyright (c) 2019, CoreRobotics.
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

#ifndef CR_CONVERSIONS_HPP_
#define CR_CONVERSIONS_HPP_


#include "Eigen/Dense"
#include "core/Types.hpp"


//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace math {


//---------------------------------------------------------------------
/*!
\class Conversion
\ingroup math

\brief This class implements math conversions.

\details
## Description
This class implements vmath conversions.
 
- Conversion::deg2rad converts degress to radians
- Conversion::rad2deg converts radians to degrees.
- Conversion::wrapToPi wraps angles to +/- \f$\pi\f$ radians.
 
## Example
This example shows usage of the math functions.
\include example_CRMath.cpp
 
## References
[1] Kreyszig, E., Advanced Engineering Mathematics, Ed.9,
John Wiley & Sons, 2011.

*/
//---------------------------------------------------------------------
class Conversion {


// Static conversion methods
public:
    
    //! Convert angles in degrees to radians
    static double deg2rad(const double i_deg) { return M_PI * i_deg / 180.0; }
    
    //! Convert angles in radians to degrees
    static double rad2deg(const double i_rad) { return 180.0 * i_rad / M_PI; }

	//! Wrap angle (rad) to +/- pi
	static double wrapToPi(double angle) {
		angle = fmod(angle + M_PI, 2 * M_PI);
		if (angle < 0) {
			angle += 2 * M_PI;
		}
		return angle - M_PI;
	}
    
    //! Wrap angle (rad) to +/- pi
    static Eigen::VectorXd wrapToPi(Eigen::VectorXd angle) {
        for (int i = 0; i < angle.size(); i++) {
            angle(i) = wrapToPi(angle(i));
        }
        return angle;
    }
    
};


}
}
// end namespace
//---------------------------------------------------------------------


#endif
