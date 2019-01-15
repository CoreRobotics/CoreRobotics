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

#ifndef CRNoiseMixture_hpp
#define CRNoiseMixture_hpp

//=====================================================================
// Includes
#include "core/CRTypes.hpp"
#include "Eigen/Dense"
#include <random>
#include <vector>
#include "CRNoiseModel.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics  {
    
//=====================================================================
/*!
 \file CRNoiseMixture.hpp
 \brief Implements a class for modeling noise as a mixture of distributions.
 */
//---------------------------------------------------------------------
/*!
 \class CRNoiseMixture
 \ingroup models
 
 \brief Implements a class for modeling noise as a mixture of distributions.
 
 \details
 ## Description
 CRNoiseMixture implements methods for sampling and modeling noise as a
 mixture of probability distributions [2-3].  Each specified noise model
 is also accompanied by a weight, indicating the probability of that
 distribution being selected for sampling of the noise.
 
 - CRNoiseMixture::add adds a noise model to the mixture
 - CRNoiseMixture::sample samples from the noise model
 - CRNoiseMixture::probability evaluates the probability
 
 ## Example
 This example demonstrates use of the CRNoiseMixture class.
 
 \include example_CRNoiseMixture.cpp
 
 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 
 [3] en.wikipedia.org/wiki/Mixture_model
 */
//=====================================================================
// Paramter structure declaration
struct [[deprecated(CR_DEPRECATED)]]CRParamNoiseMixture{
    std::vector<CoreRobotics::CRNoiseModel*> models;
    std::vector<double> weights;
};
    
//=====================================================================
class [[deprecated(CR_DEPRECATED)]] CRNoiseMixture : public CRNoiseModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRNoiseMixture(unsigned i_seed);
    CRNoiseMixture();
    
//---------------------------------------------------------------------
// Add models to the mixture
public:
    
    //! Add a distribution to the mixture model
    void add(CRNoiseModel* i_model, double i_weight);
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Sample a noise vector from the density
    using CRNoiseModel::sample;
    Eigen::VectorXd sample(void);
    
    //! Evaluate the probability from the density
    using CRNoiseModel::probability;
    double probability(Eigen::VectorXd i_x);
    
//---------------------------------------------------------------------
// Public Members
public:
    
    //! Noise model parameters
    CRParamNoiseMixture m_parameters;
    
    
private:
    
    //! hide the setParameters method and make it do nothing
    using CRNoiseModel::setParameters;
    void setParameters(void) {};
    
};

//=====================================================================
// End namespace
}


#endif
