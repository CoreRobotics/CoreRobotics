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

#include "CRParticleFilter.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor creates ...
 
 \param[in]  - i_
 \param[out] - o_
 \return     -
 */
//---------------------------------------------------------------------
CRParticleFilter::CRParticleFilter(CRMotionProbabilistic* i_motionModel,
                                   CRSensorProbabilistic* i_sensorModel,
                                   std::vector<Eigen::VectorXd> i_particles){
    
    Eigen::VectorXd a(1);
    Eigen::VectorXd b(1);
    a << 0;
    b << 1;
    
    // set the initial condition to the particle filter
    m_motionModel = i_motionModel;
    m_sensorModel = i_sensorModel;
    m_particles = i_particles;
    m_uniform = new CRNoiseUniform(a, b);
}
    
    
//=====================================================================
/*!
 Destructor.
 */
//---------------------------------------------------------------------
CRParticleFilter::~CRParticleFilter(){
    delete m_motionModel;
    delete m_sensorModel;
    delete m_uniform;
}
    
    
//=====================================================================
/*!
 This method... \n
 
 \param[in]  - i_
 \param[out] - o_
 \return     -
 */
//---------------------------------------------------------------------
void CRParticleFilter::step(Eigen::VectorXd i_u,
                            Eigen::VectorXd i_zObserved)
{
    m_particles = this->sample(i_u); // predict - importance sampling
    Eigen::VectorXd w = this->update(i_zObserved); // update - bayes rule
    for (int k = 0; k < m_weights.size(); k++){
        m_weights.at(k) = w(k);
    }
    // resample if number of effective particles is below threshold
    double Neff = 1 / (w.transpose() * w);
    if (Neff < m_Nresample){
        this->resample();
    }
}
    
    
//=====================================================================
/*!
 This method... \n
 
 \param[in]  - i_
 \param[out] - o_
 \return     -
 */
//---------------------------------------------------------------------
std::vector<Eigen::VectorXd> CRParticleFilter::sample(Eigen::VectorXd i_u)
{
    std::vector<Eigen::VectorXd> states;
    for(int k = 0; k < m_particles.size(); k++){
        m_motionModel->setState(m_particles.at(k));
        Eigen::VectorXd x = m_motionModel->motion(i_u, true);
        states.push_back(x);
    }
    return states;
}
    
    
//=====================================================================
/*!
 This method... \n
 
 \param[in]  - i_
 \param[out] - o_
 \return     -
 */
//---------------------------------------------------------------------
// std::vector<double> CRParticleFilter::update(Eigen::VectorXd i_zObserved)
Eigen::VectorXd CRParticleFilter::update(Eigen::VectorXd i_zObserved)
{
    // std::vector<double> weights;
    Eigen::VectorXd weights;
    weights.setZero(m_weights.size());
    for(int k = 0; k < m_weights.size(); k++){
        m_sensorModel->setState(m_particles.at(k));
        double p = m_sensorModel->likelihood(i_zObserved);
        weights(k) = p * m_weights.at(k);
    }
    return weights / weights.sum();
}
    
    
//=====================================================================
/*!
 This method... \n
 
 \param[in]  - i_
 \param[out] - o_
 \return     -
 */
//---------------------------------------------------------------------
void CRParticleFilter::resample()
{
    int n = m_particles.size();
    
    // cumulative sum
    std::vector<double> c;
    c.push_back(m_weights.at(0));
    for(int k = 1; k < m_weights.size(); k++){
        c.push_back(m_weights.at(k) + c.at(k-1));
    }
    
    // perform uniform random sample over (0,1)
    Eigen::VectorXd u, v;
    u.setZero(n);
    v = m_uniform->sample();
    u(0) = v(0) / double(n);
    for(int k = 1; k < u.size(); k++){
        u(k) = u(0) + double(k-1) / double(n);
    }
    
    // compute the indices for systematic resample
    Eigen::VectorXi index;
    index.setZero(n);
    int i = 0;
    for (int j = 0; j < n; j++){
        while (u(j) > c.at(i)){
            i++;
        }
        index(j) = i;
    }
    
    // resample the data from the indices and uniform weights
    std::vector<Eigen::VectorXd> x;
    for (int k = 0; k < n; k++){
        x.push_back(m_particles.at(index(k)));
        m_weights.at(k) = 1 / double(n);
    }
    m_particles = x;
    
}
    
    
//=====================================================================
/*!
 This method... \n
 
 \param[in]  - i_
 \param[out] - o_
 \return     -
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRParticleFilter::getExpectedState()
{
    Eigen::VectorXd x;
    x.setZero(m_particles.at(0).size());
    int n = m_particles.size();
    for (int k = 0; k < n; k++){
        x += m_weights.at(k) * m_particles.at(k);
    }
    return x;
}
    
    
//=====================================================================
/*!
 This method... \n
 
 \param[in]  - i_
 \param[out] - o_
 \return     -
 */
//---------------------------------------------------------------------
Eigen::MatrixXd CRParticleFilter::getExpectedCovariance()
{
    Eigen::VectorXd mu = this->getExpectedState();
    Eigen::MatrixXd Cov;
    Eigen::VectorXd e;
    Cov.setZero(m_particles.at(0).size(), m_particles.at(0).size());
    int n = m_particles.size();
    for (int k = 0; k < n; k++){
        e = m_particles.at(k) - mu;
        Cov += e * e.transpose() / double(n-1);
    }
    return Cov;
}


//=====================================================================
// End namespace
}
