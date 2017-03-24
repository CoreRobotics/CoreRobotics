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

#include "Eigen/Dense"
#include "CRNoiseDirac.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor creates a noise model.\n
 
 \param[in] in_point - the vector of the dirac point distribution
 */
//---------------------------------------------------------------------
CRNoiseDirac::CRNoiseDirac(Eigen::VectorXd in_point) {
    this->setParameters(in_point);
}
CRNoiseDirac::CRNoiseDirac() {
    Eigen::VectorXd point(1);
    point(0) = 0;
    this->setParameters(point);
}
    
    
//=====================================================================
/*!
 This method sets the paramters of the noise model.  The Dirac is a point
 distribution with P=1 of drawing a sample from the point.  This is an
 effective way to model deterministic processes.
 
 \param[in] in_point - point vector of the Dirac distribution
 */
//---------------------------------------------------------------------
void CRNoiseDirac::setParameters(Eigen::VectorXd in_point)
{
    this->m_parameters.point = in_point;
}


//=====================================================================
/*!
 This method samples a random number from the specified Dirac
 distribution.\n
 
 \return - sampled state
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRNoiseDirac::sample(void)
{
    return this->m_parameters.point;
}
    
    
//=====================================================================
/*!
 This method returns the probability of x.\n
 
 \param[in] in_x - state to evaluate
 \return - probability of in_x
 */
//---------------------------------------------------------------------
double CRNoiseDirac::probability(Eigen::VectorXd in_x)
{
    if (in_x == this->m_parameters.point){
        return 1.0;
    } else {
        return 0.0;
    }
}


//=====================================================================
// End namespace
}
