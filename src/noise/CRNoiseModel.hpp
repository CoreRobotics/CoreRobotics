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

#ifndef CRNoiseModel_hpp
#define CRNoiseModel_hpp

//=====================================================================
// Includes
#include "core/CRTypes.hpp"
#include "Eigen/Dense"
#include <random>

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics  {
    
//=====================================================================
/*!
 \file CRNoiseModel.hpp
 \brief Implements a class for modeling noise.
 */
//---------------------------------------------------------------------
/*!
 \class CRNoiseModel
 \ingroup noise
 
 \brief This class implements a noise model.
 
 \details
 ## Description
 CRNoiseModel implements methods for sampling from a distribution and
 serves as a base class to specfic distributions.  CRNoiseModel uses
 inverse transform sampling [3] to generate a state.  This requires the
 user to define an inverse cumulative density according to
 
 \f$ x = F^{-1}(P) \f$
 
 where \f$F^{-1}\f$ is the inverse cumulative density.
 
 Additionally, the CRNoiseModel uses a probability density function to 
 return the probability of a state x.  Thus the user must also specify 
 a density function of the type
 
 \f$ p = f(x) \f$.
 
 - CRNoiseModel::setParameters sets the noise model callback
 - CRNoiseModel::sample samples from the noise model
 - CRNoiseModel::probability evaluates the probability
 
 ## Example
 This example demonstrates use of the CRNoiseModel class.
 
 \include example_CRNoiseModel.cpp
 
 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 
 [3] en.wikipedia.org/wiki/Inverse_transform_sampling
 */
//=====================================================================
// ICDF Paramter structure declaration
#ifndef SWIG
struct [[deprecated(CR_DEPRECATED)]] CRParamNoiseGeneric{
#else
struct CRParamNoiseGeneric{
#endif
    Eigen::VectorXd(*icdFunction)(double);
    double(*probFunction)(Eigen::VectorXd);
};
    
//=====================================================================
#ifndef SWIG
class [[deprecated(CR_DEPRECATED)]] CRNoiseModel {
#else
class CRNoiseModel {
#endif
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRNoiseModel(unsigned i_seed);
    CRNoiseModel();
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the parameters that describe the distribution
    virtual void setParameters(Eigen::VectorXd(*i_icd)(double),
                               double(*i_prob)(Eigen::VectorXd));
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Sample a vector from the density
    virtual Eigen::VectorXd sample(void);
    
    //! Evaluate the probability from the density
    virtual double probability(Eigen::VectorXd i_x);

//---------------------------------------------------------------------
// Protected Methods
protected:
    
    //! Random seed generator
    void randomSeed(void);
    
//---------------------------------------------------------------------
// Public Members
public:
    
    //! Noise model parameters
    CRParamNoiseGeneric m_parameters;
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Seed value
    unsigned m_seed;
    
    //! Random number generator
    std::default_random_engine m_generator;
    
};

//=====================================================================
// End namespace
}


#endif
