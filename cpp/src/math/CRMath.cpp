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

#include "CRMath.hpp"
#include "Eigen/Dense"
#include "Eigen/svd"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 This method performs a forward euler integration step for the dynamical
 system:\n
 
 /f$ \dot{x} = f(t,x,u,p) /f$
 
 This equation is specified as a callback function to the integration.
 
 \param [in] dynamicSystem - the continuous time dynamic system.
 \param [in] t - the current time in seconds
 \param [in] x - the current state vector
 \param [in] u - the current input vector
 \param [in] dt - the time step to integrate over
 \return the state vector at t+dt
 
 */
//---------------------------------------------------------------------
vec CRMath::forwardEulerStep(vec(dynamicSystem)(double, vec, vec),
                                                double t,
                                                vec x,
                                                vec u,
                                                double dt){
    // forward integration step
    return x + dt*dynamicSystem(t,x,u);
}
    
    
//=====================================================================
/*!
 This method performs a Runge-Kutta integration step for the dynamical
 system:\n
 
 /f$ \dot{x} = f(t,x,u,p) /f$
 
 This equation is specified as a callback function to the integration.
 The Runga-Kutta integration is an accurate integration scheme at the
 expense of computational complexity over the forward Euler method.
 
 \param [in] dynamicSystem - the continuous time dynamic system.
 \param [in] t - the current time in seconds
 \param [in] x - the current state vector
 \param [in] u - the current input vector
 \param [in] dt - the time step to integrate over
 \return the state vector at t+dt
 
 */
//---------------------------------------------------------------------
vec CRMath::rungeKuttaStep(vec(dynamicSystem)(double, vec, vec),
                           double t,
                           vec x,
                           vec u,
                           double dt){
    // RK4 step
    Eigen::VectorXd f1 = dynamicSystem(t,x,u);
    Eigen::VectorXd f2 = dynamicSystem(t+dt/2,x+dt*f1/2,u);
    Eigen::VectorXd f3 = dynamicSystem(t+dt/2,x+dt*f2/2,u);
    Eigen::VectorXd f4 = dynamicSystem(t+dt,x+dt*f3,u);
    return x + dt/6*(f1 + 2*f2 + 2*f3 + f4);
}
    
    
//=====================================================================
/*!
 This method performs generalized matrix inversion using the SVD method\n.
 
 For a matrix A, the SVD yields:
 
 /f$ A = U \Sigma V^* /f$
 
 The generalized inverse is then:
 
 /f$ A^# = V \Sigma^{-1} U^* /f$
 
 The method utilizes the Jacobi SVD for the inverse.  For large matrices,
 this will be very slow.
 
 \param [in] A - the matrix to invert
 \param [in] tol - the tolerance placed on the singular values to determine if the matrix is singular
 \param [out] Ainv - the generalized inverse of Matrix A
 \return - a flag indicating if the matrix is singular (according to the tolerance).
            true = singular, false = not singular.
 
 */
//---------------------------------------------------------------------
bool CRMath::svdInverse(Eigen::MatrixXd A, double tol, Eigen::MatrixXd& Ainv)
{
    bool isSingular = false;
    
    // Compute the SVD of A
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    // Take the SVD
    Eigen::VectorXd sVals = svd.singularValues();
    
    // check for singular condition
    for (int i = 0; i < sVals.size(); i++){
        if (sVals(i) <= tol){
            isSingular = true;
            break;
        }
    }
    
    // Compute the generalized inverse using SVD
    Eigen::VectorXd sValsInverse = sVals.array().inverse();
    Eigen::MatrixXd SigmaInv = sValsInverse.asDiagonal();
    Ainv = svd.matrixV() * SigmaInv * svd.matrixU().transpose();
    
    
    return isSingular;
}


//=====================================================================
// End namespace
}


