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
 The constructor creates a sensor model.  The in_fcn specifies the
 observation equation.  There are 2 options for the in_fcn:\n
 
 Option 1: Deterministic Model. This function has the mathematical form:
 
 \f$ zPredict =  h(x) \f$
 
 Where \f$x\f$ is the system state and \f$zPredict\f$ is the predicted 
 sensor observation. For this case, the function prototype is
 \code
 void in_fcn(Eigen::VectorXd x,
             Eigen::VectorXd& zPredict){
    // compute zPredict from x here.
 };
 \endcode
 and the internal model type state is set to deterministic.
 
 Option 2: Probabilistic Model.  This model type is more complex and a
 generalization of option 2, and has the mathematical form:
 
 \f$p(z \mid x)\f$
 
 For this case, the function prototype is
 \code
 void in_fcn(Eigen::VectorXd x,
             Eigen::VectorXd z,
             bool s,
             Eigen::VectorXd& zPredict,
             double& probOfZ){
    // compute zPredict from x here.
    // if s == true, sample zPredict from the probability density.
    // compute probOfZ by evaluating p(z|x) = p(z|zPredict)
 };
 \endcode
 
 
 \param[in] in_fcn - a model function of the form specified above
 \param[in] in_x0 - the initial state.
 */
//---------------------------------------------------------------------
CRSensorModel::CRSensorModel(void(in_predictor)(Eigen::VectorXd,
                                                Eigen::VectorXd&),
                             Eigen::VectorXd in_x0)
{
    this->detPredictor = in_predictor;
    this->type = CR_MODEL_DETERMINISTIC;
    this->setState(in_x0);
}
CRSensorModel::CRSensorModel(void(in_predictor)(Eigen::VectorXd,
                                                bool,
                                                Eigen::VectorXd&),
                             void(in_likelihood)(Eigen::VectorXd,
                                                 Eigen::VectorXd,
                                                 double&),
                             Eigen::VectorXd in_x0){
    this->probPredictor = in_predictor;
    this->probLikelihood = in_likelihood;
    this->type = CR_MODEL_STOCHASTIC;
    this->setState(in_x0);
}


//=====================================================================
/*!
 This method simulates the measurement from the value of the underlying
 state. The sampleNoise flag can be set to simulate the sensor with
 noise sampled from the internal noise model.  If the model is 
 deterministic, the sample noise flag does nothing.\n
 
 \param[in] in_sampleNoise - a boolean flag specifying if the noise 
            model should be sampled to add noise to the measurement.
 \param[out] out_z - simulated measurement.
 */
//---------------------------------------------------------------------
void CRSensorModel::measurement(bool in_sampleNoise,
                                Eigen::VectorXd &out_z)
{
    if (this->type == CR_MODEL_DETERMINISTIC){
        (this->detPredictor)(this->state, out_z);
    } else if (this->type == CR_MODEL_STOCHASTIC){
        (this->probPredictor)(this->state, in_sampleNoise, out_z);
    }
}
    
    
//=====================================================================
/*!
 This method computes the likelihood of a measurement z from the 
 underlying state.  This evaluates the model
 
 \f$ p(z\mid x) \f$
 
 If the model is deterministic, out_p will be 1 only if z is identical
 to the estimated value.\n
 
 \param[in] in_z - the measurement to be evaluated
 \param[out] out_p - the likelihood of the measurement z
 */
//---------------------------------------------------------------------
void CRSensorModel::likelihood(Eigen::VectorXd in_z,
                               double &out_p)
{
    if (this->type == CR_MODEL_DETERMINISTIC){
        Eigen::VectorXd zPred;
        (this->detPredictor)(this->state, zPred);
        if (zPred == in_z){
            out_p = 1;
        } else {
            out_p = 0;
        }
    } else if (this->type == CR_MODEL_STOCHASTIC){
        (this->probLikelihood)(this->state, in_z, out_p);
    }
}


//=====================================================================
// End namespace
}
