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

#ifndef CRNoiseUniform_hpp
#define CRNoiseUniform_hpp

//=====================================================================
// Includes
#include "core/CRTypes.hpp"
#include "Eigen/Dense"
#include <random>
#include "CRNoiseModel.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics  {
    
//=====================================================================
/*!
 \file CRNoiseUniform.hpp
 \brief Implements a class for modeling uniform noise.
 */
//---------------------------------------------------------------------
/*!
 \class CRNoiseUniform
 \ingroup noise
 
 \brief Implements a class for modeling uniform noise.
 
 \details
 ## Description
 CRNoiseUniform implements methods for modeling and sampling uniform
 noise.  The uniform distribution is defined by a lower bound \f$a\f$
 and upper bound \f$b\f$ on the range of the sampled state [3].  Every
 state in this range has equal probability of being sampled.
 
 - CRNoiseUniform::setParameters sets the parameters of the noise model
 - CRNoiseUniform::sample samples from the noise model
 - CRNoiseUniform::probability evaluates the probability
 
 ## Example
 This example demonstrates use of the CRNoiseUniform class.
 
 \include example_CRNoiseUniform.cpp
 
 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press, 2006.
 \n\n
 
 [3] en.wikipedia.org/wiki/Uniform_distribution_(continuous)
 \n\n
 */
//=====================================================================
// Paramter structure declaration
struct [[deprecated(CR_DEPRECATED)]]CRParamNoiseUniform{
    Eigen::VectorXd a;
    Eigen::VectorXd b;
};
    
//=====================================================================
class [[deprecated(CR_DEPRECATED)]] CRNoiseUniform : public CRNoiseModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRNoiseUniform(Eigen::VectorXd i_a,
                   Eigen::VectorXd i_b,
                    unsigned i_seed);
    CRNoiseUniform(Eigen::VectorXd i_a,
                   Eigen::VectorXd i_b);
    CRNoiseUniform();
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the parameters that describe the distribution
    using CRNoiseModel::setParameters;
    void setParameters(Eigen::VectorXd i_a,
                       Eigen::VectorXd i_b);
    
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
    CRParamNoiseUniform m_parameters;
    
};

//=====================================================================
// End namespace
}


#endif
