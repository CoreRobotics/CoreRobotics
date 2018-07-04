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

*/
//=====================================================================

#ifndef NoiseGaussian_hpp
#define NoiseGaussian_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include <random>
#include "NoiseModel.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file NoiseGaussian.hpp
 \brief Implements a class for modeling Gaussian noise.
 */
//---------------------------------------------------------------------
/*!
 \class NoiseGaussian
 \ingroup models
 
 \brief Implements a class for modeling Gaussian noise.
 
 \details
 ## Description
 NoiseGaussian implements methods for sampling from and modeling 
 multivariate Gaussian noise (see [1-3]). The Gaussian is completely
 defined by a mean \f$\mu\f$ and covariance \f$\Sigma\f$.
 
 - NoiseGaussian::setParameters sets the parameters of the noise model
 - NoiseGaussian::sample samples from the noise model
 - NoiseGaussian::probability evaluates the probability
 
 ## Example
 This example demonstrates use of the NoiseGaussian class.
 
 \include example_NoiseGaussian.cpp
 
 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 
 [3] en.wikipedia.org/wiki/Multivariate_normal_distribution
 */
//=====================================================================
// Paramter structure declaration
struct CRParamNoiseGaussian{
    Eigen::MatrixXd cov;
    Eigen::MatrixXd covInv;
    Eigen::VectorXd mean;
};
    
//=====================================================================
class NoiseGaussian : public NoiseModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    NoiseGaussian(Eigen::MatrixXd i_cov,
                    Eigen::VectorXd i_mean,
                    unsigned i_seed);
    NoiseGaussian(Eigen::MatrixXd i_cov,
                    Eigen::VectorXd i_mean);
    NoiseGaussian();
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the parameters that describe the distribution
    using NoiseModel::setParameters;
    void setParameters(Eigen::MatrixXd i_cov,
                       Eigen::VectorXd i_mean);
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Sample a noise vector from the density
    using NoiseModel::sample;
    Eigen::VectorXd sample(void);
    
    //! Evaluate the probability from the density
    using NoiseModel::probability;
    double probability(Eigen::VectorXd i_x);
    
//---------------------------------------------------------------------
// Public Members
public:
    
    //! Noise model parameters
    CRParamNoiseGaussian m_parameters;
    
};

//=====================================================================
// End namespace
}


#endif
