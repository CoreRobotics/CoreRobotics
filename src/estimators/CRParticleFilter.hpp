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

#ifndef CRParticleFilter_hpp
#define CRParticleFilter_hpp

//=====================================================================
// Includes
#include <vector>
#include "Eigen/Dense"
#include "CRNoiseUniform.hpp"
#include "CRSensorProbabilistic.hpp"
#include "CRMotionProbabilistic.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRParticleFilter.hpp
 \brief Implements a particle filter.
 */
//---------------------------------------------------------------------
/*!
 \class CRParticleFilter
 \ingroup estimators
 
 \brief This class implements a particle filter for nonlinear,
 non-Gaussian state estimation.
 
 \details
 ## Description
 
 
 These methods are available:
 - CRParticleFilter::setState sets ...
 
 ## Example
 This example demonstrates use of the CRParticleFilter class.
 \include test_CRParticleFilter.cpp
 
 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 */
//=====================================================================
class CRParticleFilter {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRParticleFilter(CRMotionProbabilistic* i_motionModel,
                     CRSensorProbabilistic* i_sensorModel,
                     std::vector<Eigen::VectorXd> i_particles);
    
    //! Class destructor
    ~CRParticleFilter();
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Perform a filter step
    void step(Eigen::VectorXd i_u,
              Eigen::VectorXd i_zObserved);
    
    //! Progpogate the particles forward
    std::vector<Eigen::VectorXd> sample(Eigen::VectorXd i_u);
    
    //! Update the weights using likelihood
    Eigen::VectorXd update(Eigen::VectorXd i_zObserved);
    
    //! Resample step
    void resample();
    
    //! Compute the mean
    Eigen::VectorXd getExpectedState();
    
    //! Compute the covariance
    Eigen::MatrixXd getExpectedCovariance();
    
//---------------------------------------------------------------------
// Public Members
public:
    
    //! Motion Model
    CRMotionProbabilistic* m_motionModel;
    
    //! Sensor Model
    CRSensorProbabilistic* m_sensorModel;
    
    //! vector of particles
    std::vector<Eigen::VectorXd> m_particles;
    
    //! vector of weights
    std::vector<double> m_weights;
    
    //! critical number of particles
    double m_Nresample;
    
    //! Uniform random noise
    CRNoiseUniform* m_uniform;
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
};

//=====================================================================
// End namespace
}


#endif
