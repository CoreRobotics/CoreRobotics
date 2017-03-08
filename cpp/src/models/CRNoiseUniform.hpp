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

#ifndef CRNoiseUniform_hpp
#define CRNoiseUniform_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include <random>
#include "CRNoiseModel.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRNoiseUniform.hpp
 \brief Implements a class for modeling uniform noise.
 */
//---------------------------------------------------------------------
/*!
 \class CRNoiseUniform
 \ingroup models
 
 \brief Implements a class for modeling uniform noise.
 
 \details
 \section Description
 CRNoiseUniform implements methods for modeling and sampling uniform
 noise.  The uniform distribution is defined by a lower bound \f$a\f$
 and upper bound \f$b\f on the range of the sampled state [3].  Every
 state in this range has equal probability of being sampled.
 
 \section Example
 This example demonstrates use of the CRNoiseUniform class.
 
 \code
 #include <iostream>
 #include "CoreRobotics.hpp"
 
 // Use the CoreRobotics namespace
 using namespace CoreRobotics;
 
 void main(void){
 
     std::cout << "*************************************\n";
     std::cout << "Demonstration of test_CRNoiseUniform.\n";
     
     // define the uniform properties
     Eigen::VectorXd a(1);
     Eigen::VectorXd b(1);
     a << 2; // lower bound on the domain
     b << 8; // upper bound on the domain
     
     // initialize a noise model
     CRNoiseUniform uniformNoise = CRNoiseUniform();
     uniformNoise.setParameters(a, b);
     
     // initialize a vector to sample into
     Eigen::VectorXd v(1);
     
     const int nrolls=10000;  // number of experiments
     const int nstars=100;     // maximum number of stars to distribute
     int p[10]={};
     
     // sample the distribution
     for (int i=0; i<nrolls; ++i) {
         uniformNoise.sample(v);
         if ((v(0)>=0.0)&&(v(0)<10.0)) ++p[int(v(0))];
     }
     
     // print out the result with stars to indicate density
     std::cout << std::fixed; std::cout.precision(1);
     for (int i=0; i<10; ++i) {
         std::cout << i << " - " << (i+1) << ": ";
         std::cout << std::string(p[i]*nstars/nrolls,'*') << std::endl;
     }
 }
 \endcode
 
 \section References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 
 [3] en.wikipedia.org/wiki/Uniform_distribution_(continuous)
 */
//=====================================================================
// Paramter structure declaration
struct uniformParam{
    Eigen::VectorXd a;
    Eigen::VectorXd b;
};
    
//=====================================================================
class CRNoiseUniform : public CRNoiseModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRNoiseUniform(Eigen::VectorXd a,
                   Eigen::VectorXd b,
                    unsigned seed);
    CRNoiseUniform(Eigen::VectorXd a,
                   Eigen::VectorXd b);
    CRNoiseUniform();
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the parameters that describe the distribution
    using CRNoiseModel::setParameters;
    void setParameters(Eigen::VectorXd a,
                       Eigen::VectorXd b);
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Sample a noise vector from the density
    using CRNoiseModel::sample;
    void sample(Eigen::VectorXd &x);
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Noise model type
    uniformParam parameters;
    
    //! Seed value
    unsigned seed;
    
    //! Random number generator
    std::default_random_engine generator;
    
};

//=====================================================================
// End namespace
}


#endif
