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

#ifndef CRNoiseModel_hpp
#define CRNoiseModel_hpp

//=====================================================================
// Includes
#include "Eigen/Dense"
#include <random>

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRNoiseModel.hpp
 \brief Implements a class for modeling noise.
 */
//---------------------------------------------------------------------
/*!
 \class CRNoiseModel
 \ingroup models
 
 \brief This class implements a noise model.
 
 \details
 \section Description
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
 
 \section Example
 This example demonstrates use of the CRNoiseModel class.
 
 \code
 
 #include <iostream>
 #include "CoreRobotics.hpp"
 
 // Use the CoreRobotics namespace
 using namespace CoreRobotics;
 
 // declare an inverse cumulative distribution - this is the invserse
 // CDF for a triangular distribution from [0,1].
 Eigen::VectorXd icdf(double P){
     Eigen::VectorXd v(1);
     v(0) = sqrt(P);
     return v;
 }
 
 // declare the probability density - this is the traditional density
 // defined by a distribution
 double pdensity(Eigen::VectorXd x){
     return 2*x(0);
 }
 
 void main(void){
 
     std::cout << "*************************************\n";
     std::cout << "Demonstration of CRNoiseModel.\n";
     
     // initialize a noise model
     CRNoiseModel genericNoise = CRNoiseModel();
     genericNoise.setParameters(*icdf,*pdensity);
     
     
     // initialize parameters for experiments
     const int nrolls=10000;  // number of experiments
     const int nstars=100;    // maximum number of stars to distribute
     const int nintervals=10; // number of intervals
     int p[10]={};
     
     // sample the distribution
     for (int i=0; i<nrolls; ++i) {
         Eigen::VectorXd v = genericNoise.sample();
         ++p[int(nintervals*v(0))];
     }
     
     // print out the result with stars to indicate density
     std::cout << std::fixed; std::cout.precision(1);
     for (int i=0; i<nintervals; ++i) {
         std::cout << float(i)/nintervals << " - " << float(i+1)/nintervals << ": ";
         printf("%2i - %2i | ",i,i+1);
         Eigen::VectorXd point(1);
         point << double(i);
         double prob = genericNoise.probability(point);
         printf("%4.1f | ",prob);
         std::cout << std::string(p[i]*nstars/nrolls,'*') << std::endl;
     }
 }
 
 \endcode
 
 \section References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 
 [3] en.wikipedia.org/wiki/Inverse_transform_sampling
 */
//=====================================================================
// ICDF Paramter structure declaration
struct CRParamNoiseGeneric{
    Eigen::VectorXd(*icdFunction)(double);
    double(*probFunction)(Eigen::VectorXd);
};
    
//=====================================================================
class CRNoiseModel {
    
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
// Protected Members
protected:
    
    //! Noise model type
    CRParamNoiseGeneric m_parameters;
    
    //! Seed value
    unsigned m_seed;
    
    //! Random number generator
    std::default_random_engine m_generator;
    
};

//=====================================================================
// End namespace
}


#endif
