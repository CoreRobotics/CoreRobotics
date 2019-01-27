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

#include "CRSensorProbabilistic.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics  {
    
    
//=====================================================================
/*!
 The constructor creates a sensor model, and requires 2 callback 
 functions to be supplied.
 
 Callback 1: i_predictor
 
 The i_predictor specifies the observation equation:\n
 
 \f$ zPredict =  h(x,w) \f$
 
 where \f$x\f$ is the system state, \f$w\f$ is noise and \f$zPredict\f$
 is the predicted sensor observation. The callback function prototype 
 is thus
 \code
 Eigen::VectorXd i_predictor(Eigen::VectorXd x,
                              boolean sample){
    // compute zPredict from x here.
    if (sample) {
        // sample from the noise distribution to generate
        // zPredictWithNoise
        return zPredictWithNoise;
    } else {
        return zPredict;
    }
 
 };
 \endcode
 
 
 Callback 2: i_likelihood
 
 The i_likelihood specifies the likelihood:\n
 
 \f$ p =  Pr(zObserved \mid zPredict) \f$
 
 where \f$zPredict\f$ is the predicted observation, \f$zObserved\f$ is
 an actual measurement and \f$p\f$ is the probability of observing 
 zObserved given zPredict. The callback function prototype is thus
 \code
 double i_likelihood(Eigen::VectorXd zObserved,
                      Eigen::VectorXd zPredict){
    // compute p = Pr(zObserved | zPredict)
    return p;
 };
 \endcode
 
 
 \param[in] i_predictor - prediction callback function.
 \param[in] i_likelihood - likelihood callback function.
 \param[in] i_x0 - the initial state.
 */
//---------------------------------------------------------------------
CRSensorProbabilistic::CRSensorProbabilistic(Eigen::VectorXd(i_predictor)(Eigen::VectorXd,
                                                                           bool),
                                             double(i_likelihood)(Eigen::VectorXd,
                                                                   Eigen::VectorXd),
                                             Eigen::VectorXd i_x0){
    this->m_measPredictFcn = i_predictor;
    this->m_measLikelihoodFcn = i_likelihood;
    this->setState(i_x0);
}


//=====================================================================
/*!
 This method simulates the measurement from the value of the underlying
 state. The sampleNoise flag can be set to simulate the sensor with
 noise sampled from the internal noise model.\n
 
 \param[in] i_sampleNoise - an (optional) boolean flag specifying if
            the noise model should be sampled to add noise to the 
            simulated measurement.
 \return - simulated measurement.
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRSensorProbabilistic::measurement(bool i_sampleNoise)
{
    return (this->m_measPredictFcn)(this->m_state, i_sampleNoise);
}
    
Eigen::VectorXd CRSensorProbabilistic::measurement(void)
{
    return (this->m_measPredictFcn)(this->m_state, false);
}
    
    
//=====================================================================
/*!
 This method computes the likelihood of a measurement z from the 
 underlying state.  This evaluates the model
 
 \f$ p(zObserved \mid zPredict) \f$
 
 \param[in] i_zObserved - the actual measurement to be evaluated.
 \return - the likelihood of zObserved.
 */
//---------------------------------------------------------------------
double CRSensorProbabilistic::likelihood(Eigen::VectorXd i_zObserved)
{
    Eigen::VectorXd zPredict = this->measurement(false);
    return (this->m_measLikelihoodFcn)(this->m_state, i_zObserved);
}


//=====================================================================
// End namespace
}
