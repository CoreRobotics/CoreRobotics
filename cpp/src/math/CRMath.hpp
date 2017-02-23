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
 \version 0.0
 
 */
//=====================================================================

#ifndef CRMath_hpp
#define CRMath_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {

    const double PI = 3.1415926535897932384626433832795;

    //=====================================================================
    /*!
    \file CRMath.hpp
    \brief Implements utility math functions.
    */
    //---------------------------------------------------------------------
    /*!
    \class CRMath
    \ingroup math

    \brief This class implements various math related helper functions.

    \details
    \section Description
    CRMath implements math static helper functions.
     
     
    References:
    
    [1] Kreyszig, E., Advanced Engineering Mathematics, Ed.9,
    John Wiley & Sons, 2011.

    */

    //=====================================================================
    class CRMath {

    //---------------------------------------------------------------------
    // Static conversion methods
    public:
        
        //! Convert angles in degrees to radians
        static double deg2rad(double deg) { return PI * deg / 180.0; }
        
        //! Convert angles in radians to degrees
        static double rad2deg(double rad) { return 180.0 * rad / PI; }
        
        
    //---------------------------------------------------------------------
    // Numerical integration routines
    public:
        
        //! Forward euler integration
        static Eigen::VectorXd forwardEulerStep(
            Eigen::VectorXd(dyanmicSystem)(double, Eigen::VectorXd, Eigen::VectorXd),
            double t, Eigen::VectorXd x, Eigen::VectorXd u, double dt);
        
        //! Runge-Kutta 4th order integration
        static Eigen::VectorXd rungeKuttaStep(
             Eigen::VectorXd(dyanmicSystem)(double, Eigen::VectorXd, Eigen::VectorXd),
             double t, Eigen::VectorXd x, Eigen::VectorXd u, double dt);
        
        
    };


//=====================================================================
// End namespace
}

#endif /* CRMath_hpp */
