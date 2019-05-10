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
\author  Cameron Devine

*/
//=====================================================================

#include "CRKalmanFilter.hpp"


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {


//=====================================================================
/*!
 This method steps the Kalman Filter forward in time with the given input
 and measurement.
 
 \param[in] i_input             the input to the dynamical system
 \param[in] i_measurement       the noisy measurement taken of the
                                system
 \return                        a CRResult flag indicating if the 
                                operation encountered a singularity
 */
//---------------------------------------------------------------------
CRResult CRKalmanFilter::step(Eigen::VectorXd i_input,
                              Eigen::VectorXd i_measurement) {
	CRResult result = CR_RESULT_SUCCESS;
	if (std::isnan(this->m_dt)) { // If the system is discrete time
		Eigen::VectorXd xhat = this->m_A * this->m_x + this->m_B * i_input;
		Eigen::MatrixXd SigmaHat = this->m_A * this->m_Sigma * this->m_A.transpose() + this->m_Q;
		Eigen::MatrixXd S = this->m_R + this->m_C * SigmaHat * this->m_C.transpose();
		Eigen::MatrixXd Sinv;
		result = CRMath::svdInverse(S, this->m_tol, Sinv);
		Eigen::MatrixXd K = SigmaHat * this->m_C.transpose() * Sinv;
		this->m_x = xhat + K * (i_measurement - this->m_C * xhat);
		this->m_Sigma = SigmaHat - K * S * K.transpose();
	} else { // If the system is continuous time
		Eigen::MatrixXd K = this->m_Sigma * this->m_C.transpose() * this->m_Rinv;
		this->m_x += this->m_dt * (this->m_A * this->m_x
		                         + this->m_B * i_input
		                         + K * (i_measurement - this->m_C * this->m_x));
		this->m_Sigma += this->m_dt * (this->m_A * this->m_Sigma + this->m_Sigma * this->m_A.transpose()
		                             + this->m_Q
		                             - K * this->m_R * K.transpose());
	}
	return result;
}

//=====================================================================
// End namespace
}
