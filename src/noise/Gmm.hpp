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

#ifndef Gmm_hpp
#define Gmm_hpp

//=====================================================================
// Includes
#include <random>
#include <vector>
#include "Eigen/Dense"
#include "NoiseMixture.hpp"
#include "NoiseGaussian.hpp"

//=====================================================================
// CoreRobotics namespace
namespace cr {
    
//=====================================================================
/*!
 \file Gmm.hpp
 \brief Implements a class for Gaussian mixture models.
 */
//---------------------------------------------------------------------
/*!
 \class Gmm
 \ingroup models
 
 \brief Implements a class for Gaussian mixture models.
 
 \details
 ## Description
 Gmm implements methods for sampling and modeling noise as a
 mixture of Gaussian distributions [2-3].  Each specified Gaussian
 is also accompanied by a weight, indicating the probability of that
 distribution being selected for sampling of the noise.
 
 - Gmm::add adds a noise model to the mixture
 - Gmm::sample samples from the noise model
 - Gmm::probability evaluates the probability
 - Gmm::regression performs Gaussian mixture regression
 
 ## Example
 This example demonstrates use of the Gmm class.
 
 \include example_Gmm.cpp
 
 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 
 [3] H. Sung, "Gaussian Mixture Regression and Classification", PhD Thesis,
 Rice University, 2004. \n\n
 */
//=====================================================================
// Paramter structure declaration
struct CRParamGaussianMixture{
    std::vector<cr::NoiseGaussian> models;
    std::vector<double> weights;
};
    
//=====================================================================
class Gmm : public NoiseMixture {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    Gmm(unsigned i_seed);
    Gmm();

	//! Class destructor
	~Gmm();
    
//---------------------------------------------------------------------
// Add models to the mixture
public:
    
    //! Add a distribution to the mixture model
    void add(NoiseGaussian i_model, double i_weight);
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Sample a noise vector from the density
    using NoiseMixture::sample;
    Eigen::VectorXd sample(void);
    
    //! Evaluate the probability from the density
    using NoiseMixture::probability;
    double probability(Eigen::VectorXd i_x);
    
    //! Perform Gaussian Mixture Regression (GMR)
    void regression(Eigen::VectorXd i_x,
                    Eigen::VectorXi i_inputIndices,
                    Eigen::VectorXi i_outputIndices,
                    Eigen::VectorXd& o_mean,
                    Eigen::MatrixXd& o_covariance);
    
//---------------------------------------------------------------------
// Public Members
public:
    
    //! Noise model parameters
    CRParamGaussianMixture m_parameters;
    
private:
    
    //! Evaluate the multivariate normal dist
    double mvnpdf(Eigen::VectorXd i_x,
                  Eigen::VectorXd i_mean,
                  Eigen::MatrixXd i_covariance);
    
};

//=====================================================================
// End namespace
}


#endif
