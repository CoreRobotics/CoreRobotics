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

#include "CRTrajectoryGenerator.hpp"
#include "CRMath.hpp"
#include <math.h>
#include <iostream>


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 Constructor.\n
 */
//---------------------------------------------------------------------
CRTrajectoryGenerator::CRTrajectoryGenerator()
{ }


//=====================================================================
/*!
 This method computes the trajectory for the specified conditions.  When
 this method is called, the internal clock is reset to time = 0.\n
 
 \param[in]     i_x0 - initial state vector (position)
 \param[in]     i_x0 - initial state 1st derivative (velocity)
 \param[in]     i_a0 - initial state 2nd derivative (acceleration)
 \param[in]     i_xf - final state vector (position)
 \param[in]     i_xf - final state 1st derivative (velocity)
 \param[in]     i_af - final state 2nd derivative (acceleration)
 \param[in]     i_tf - final time
 \return        CoreRobotics::CRResult indicator
 */
//---------------------------------------------------------------------
CRResult CRTrajectoryGenerator::solve(Eigen::VectorXd i_x0,
                                      Eigen::VectorXd i_v0,
                                      Eigen::VectorXd i_a0,
                                      Eigen::VectorXd i_xf,
                                      Eigen::VectorXd i_vf,
                                      Eigen::VectorXd i_af,
                                      double i_tf)
{
    
    // indicator if solution is singular
    CRResult result = CR_RESULT_SUCCESS;         // break the algorithm if it is singular
    
    // Set the internal time to the specified final time
    this->m_tf = i_tf;
    
    // Compute the inv(A) matrix
    double t = i_tf;
    Eigen::Matrix<double, 6, 6> Ainv;
    Ainv << 1,      0,          0,       0,      0,         0,
             0,      1,          0,       0,      0,         0,
             0,      0,        0.5,       0,      0,         0,
        -10/pow(t,3), -6/pow(t,2),   -3/(2*t),  10/pow(t,3), -4/pow(t,2),   1/(2*t),
         15/pow(t,4),  8/pow(t,3),  3/(2*pow(t,2)), -15/pow(t,4),  7/pow(t,3),    -1/pow(t,2),
         -6/pow(t,5), -3/pow(t,4), -1/(2*pow(t,3)),   6/pow(t,5), -3/pow(t,4), 1/(2*pow(t,3));
    
    // Compute the b matrix
    int n = i_x0.size();
    Eigen::Matrix<double, 6, Eigen::Dynamic> b;
    b.setZero(6, n);
    b << i_x0.transpose(), i_v0.transpose(), i_a0.transpose(), i_xf.transpose(), i_vf.transpose(), i_af.transpose();
    
    // Find the coefficient matrix
    this->m_X = Ainv * b;
    
    // start the internal clock
    this->m_timer.startTimer();

	// return result
	return result;
}
    
    
//=====================================================================
/*!
 This method computes the values of the trajectory at time i_t (s).\n
 
 \param[in]     i_t - the time at which to evaluate.
 \param[out]    o_x - the output at time t
 \param[out]    o_v - the 1st derivatrive at time t
 \param[out]    o_a - the 2nd derivatrive at time t
 \param[out]    o_j - the 3rd derivatrive at time t
 */
//---------------------------------------------------------------------
void CRTrajectoryGenerator::step(double i_t,
                                 Eigen::VectorXd& o_x,
                                 Eigen::VectorXd& o_v,
                                 Eigen::VectorXd& o_a,
                                 Eigen::VectorXd& o_j)
{
    // Limit the defined time
    double t = i_t;
    if (t >= this->m_tf) {
        t = this->m_tf;
    }
    
    //! Velocity Coefficients
    Eigen::Matrix<double, 1, 5> vCoeff;
    vCoeff << 1.0, 2.0, 3.0, 4.0, 5.0;
    
    //! Acceleration Coefficients
    Eigen::Matrix<double, 1, 4> aCoeff;
    aCoeff << 2.0, 6.0, 12.0, 20.0;
    
    //! Acceleration Coefficients
    Eigen::Matrix<double, 1, 3> jCoeff;
    jCoeff << 6, 24, 60;
    
    //! Compute the polynomials in time
    Eigen::Matrix<double, 6, 1> T;
    T << pow(t,0), pow(t,1), pow(t,2), pow(t,3), pow(t,4), pow(t,5);
    
    // Initialize the output values to zero
    int n = this->m_X.cols();
    o_x.setZero(n,1);
    o_v.setZero(n,1);
    o_a.setZero(n,1);
    o_j.setZero(n,1);
    
    // Now compute the output for each element
    for (int i=0; i<n; i++){
        
        // Get the coefficient vector
        Eigen::Matrix<double, 1, 6> X;
        X = this->m_X.col(i).transpose();
        
        // std::cout << "t = " << t << "\n\n" << "X = " << X << "\n\n" << "T = " << T << "\n\n";
        
        o_x(i) = X * T; // pos
        o_v(i) = vCoeff.cwiseProduct(X.tail(5)) * T.head(5); // vel
        o_a(i) = aCoeff.cwiseProduct(X.tail(4)) * T.head(4); // accel
        o_j(i) = jCoeff.cwiseProduct(X.tail(3)) * T.head(3); // jerk
    }
}
    
    
//=====================================================================
/*!
 This method computes the values of the trajectory for the elapsed time
 since the internal clock was started on the CRTrajectoryGenerator::solve
 call.\n
 
 \param[out]    o_x - the output at time t
 \param[out]    o_v - the 1st derivatrive at time t
 \param[out]    o_a - the 2nd derivatrive at time t
 \param[out]    o_j - the 3rd derivatrive at time t
 */
//---------------------------------------------------------------------
void CRTrajectoryGenerator::step(Eigen::VectorXd& o_x,
                                 Eigen::VectorXd& o_v,
                                 Eigen::VectorXd& o_a,
                                 Eigen::VectorXd& o_j)
{
    // Get the elapsed time since the solver was called
    double t = this->m_timer.getElapsedTime();
    
    // Compute the trajectory at the internal clock time
    CRTrajectoryGenerator::step(t, o_x, o_v, o_a, o_j);
}


//=====================================================================
// End namespace
}
