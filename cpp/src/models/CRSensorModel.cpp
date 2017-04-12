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

#include "CRSensorModel.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor creates a sensor model.  The i_predictor specifies 
 the observation equation:\n
 
 \f$ zPredict =  h(x) \f$
 
 where \f$x\f$ is the system state and \f$zPredict\f$ is the predicted 
 sensor observation. The callback function prototype is thus
 \code
 Eigen::VectorXd i_predictor(Eigen::VectorXd x){
    // compute zPredict from x here.
    return zPredict;
 };
 \endcode
 
 \param[in] i_predictor - a model function of the form specified above
 \param[in] i_x0 - the initial state.
 */
//---------------------------------------------------------------------
CRSensorModel::CRSensorModel(Eigen::VectorXd(i_predictor)(Eigen::VectorXd),
                             Eigen::VectorXd i_x0)
{
    this->m_measPredictFcn = i_predictor;
    this->setState(i_x0);
}

// overloaded constructor for initializing derived classes
CRSensorModel::CRSensorModel() { }


//=====================================================================
/*!
 This method simulates the measurement from the value of the underlying
 state.\n
 
 \return - simulated measurement (z).
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRSensorModel::measurement(void)
{
    return (this->m_measPredictFcn)(this->m_state);
}


//=====================================================================
// End namespace
}
