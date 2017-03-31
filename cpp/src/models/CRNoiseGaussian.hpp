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

#ifndef CRNoiseGaussian_hpp
#define CRNoiseGaussian_hpp

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
 \file CRNoiseGaussian.hpp
 \brief Implements a class for modeling Gaussian noise.
 */
//---------------------------------------------------------------------
/*!
 \class CRNoiseGaussian
 \ingroup models
 
 \brief Implements a class for modeling Gaussian noise.
 
 \details
 \section Description
 CRNoiseGaussian implements methods for sampling from and modeling 
 multivariate Gaussian noise (see [1-3]). The Gaussian is completely
 defined by a mean \f$\mu\f$ and covariance \f$\Sigma\f$.
 
 \section Example
 This example demonstrates use of the CRNoiseGaussian class.
 
 \code
 
 #include <iostream>
 #include "CoreRobotics.hpp"
 
 // Use the CoreRobotics namespace
 using namespace CoreRobotics;
 
 void main(void){
 
     std::cout << "*************************************\n";
     std::cout << "Demonstration of CRNoiseGaussian.\n";
     
     // define the Gaussian properties
     Eigen::Vector2d mean;
     mean << 5, 5;
     Eigen::Matrix2d cov;
     cov << 3, 0, 0, 3;
     
     // initialize a noise model
     CRNoiseGaussian normalNoise = CRNoiseGaussian();
     normalNoise.setParameters(cov, mean);
     
     // initialize parameters for experiments
     const int nrolls=10000;  // number of experiments
     const int nstars=100;    // maximum number of stars to distribute
     int p[10]={};
     
     // sample the distribution
     for (int i=0; i<nrolls; ++i) {
         Eigen::VectorXd v = normalNoise.sample();
         if ((v(0)>=0.0)&&(v(0)<10.0)) ++p[int(v(0))];
     }
     
     // print out the result with stars to indicate density
     std::cout << std::fixed; std::cout.precision(1);
     for (int i=0; i<10; ++i) {
         printf("%2i - %2i | ",i,i+1);
         Eigen::VectorXd point(2);
         point << double(i), 5;
         double prob = normalNoise.probability(point);
         printf("%6.4f | ",prob);
         std::cout << std::string(p[i]*nstars/nrolls,'*') << std::endl;
     }
 }
 
 \endcode
 
 \section References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 
 [3] en.wikipedia.org/wiki/Multivariate_normal_distribution
 */
//=====================================================================
// Paramter structure declaration
struct gaussianParam{
    Eigen::MatrixXd cov;
    Eigen::MatrixXd covInv;
    Eigen::VectorXd mean;
};
    
//=====================================================================
class CRNoiseGaussian : public CRNoiseModel {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRNoiseGaussian(Eigen::MatrixXd in_cov,
                    Eigen::VectorXd in_mean,
                    unsigned in_seed);
    CRNoiseGaussian(Eigen::MatrixXd in_cov,
                    Eigen::VectorXd in_mean);
    CRNoiseGaussian();
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the parameters that describe the distribution
    using CRNoiseModel::setParameters;
    void setParameters(Eigen::MatrixXd in_cov,
                       Eigen::VectorXd in_mean);
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Sample a noise vector from the density
    using CRNoiseModel::sample;
    Eigen::VectorXd sample(void);
    
    //! Evaluate the probability from the density
    using CRNoiseModel::probability;
    double probability(Eigen::VectorXd in_x);
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! Noise model type
    gaussianParam m_parameters;
    
};

//=====================================================================
// End namespace
}


#endif
