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
//=====================================================================

#ifndef Integration_hpp
#define Integration_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include "core/Types.hpp"


//=====================================================================
// CoreRobotics namespace
namespace cr {

//=====================================================================
/*!
\file Integration.hpp
\brief This class implements numerical integration methods.
*/
//---------------------------------------------------------------------
/*!
\class Integration
\ingroup math

\brief This class implements numerical integration methods.

\details
## Description
This class implements numerical integration methods.
 
- Integration::forwardEulerStep performs forward Euler integration.
- Integration::rungeKuttaStep performs 4th order Runga-Kutta integration.
 
## Example
This example shows usage of the math functions.
\include example_CRMath.cpp
 
## References
[1] Kreyszig, E., Advanced Engineering Mathematics, Ed.9,
John Wiley & Sons, 2011.

*/

//=====================================================================
class Integration {
    
#ifndef SWIG
//---------------------------------------------------------------------
// Numerical integration routines
public:
    
    //! Forward euler integration
    static Eigen::VectorXd forwardEulerStep(Eigen::VectorXd(i_dynamicSystem)(double,
                                                                             Eigen::VectorXd,
                                                                             Eigen::VectorXd),
                                            double i_t,
                                            Eigen::VectorXd i_x,
                                            Eigen::VectorXd i_u,
                                            double i_dt);
    
    static Eigen::VectorXd forwardEulerStep(std::function<Eigen::VectorXd(double,
                                                                          Eigen::VectorXd,
                                                                          Eigen::VectorXd)> i_dynamicSystem,
                                            double i_t,
                                            Eigen::VectorXd i_x,
                                            Eigen::VectorXd i_u,
                                            double i_dt);
    
    
    //! Runge-Kutta 4th order integration
    static Eigen::VectorXd rungeKuttaStep(Eigen::VectorXd(i_dynamicSystem)(double,
                                                                           Eigen::VectorXd,
                                                                           Eigen::VectorXd),
                                          double i_t,
                                          Eigen::VectorXd i_x,
                                          Eigen::VectorXd i_u,
                                          double i_dt);
    
    static Eigen::VectorXd rungeKuttaStep(std::function<Eigen::VectorXd(double,
                                                                        Eigen::VectorXd,
                                                                        Eigen::VectorXd)> i_dynamicSystem,
                                            double i_t,
                                            Eigen::VectorXd i_x,
                                            Eigen::VectorXd i_u,
                                            double i_dt);
#endif
    
};


//=====================================================================
// End namespace
}

#endif /* CRMath_hpp */
