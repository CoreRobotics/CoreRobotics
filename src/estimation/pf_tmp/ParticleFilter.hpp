/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_PARTICLE_FILTER_HPP_
#define CR_PARTICLE_FILTER_HPP_

#include <vector>
#include "Eigen/Dense"
#include "noise/Discrete.hpp"

namespace cr {
namespace estimation {

//------------------------------------------------------------------------------
/*!
 \class ParticleFilter
 \ingroup estimators
 
 \brief This class implements a particle filter for nonlinear,
 non-Gaussian state estimation.
 
 \details
 ## Description
 
 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press,
 2006. \n\n
 */
//------------------------------------------------------------------------------
class ParticleFilter {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRParticleFilter(const model::MotionStatistical& i_motion,
                     const model::SensorStatistical& i_sensor,
                     const noise::Particle& i_state);
    
    //! Class destructor
    virtual ~CRParticleFilter() = delete;
	
	//! This function steps the callback and updates the state.
	void step() override;

    //! Set the state estimate
	void setState(const noise::Particle& i_x) { m_state = i_x; }

	//! Get the state estimate
	const noise::Particle& getState() { return m_state; }

    //! Prediction step
	void predict(const Eigen::VectorXd& i_u, noise::Particle& o_x);

	//! Correction step
	void correct(const Eigen::VectorXd& i_z, noise::Particle& o_x);

    //! Resample step
    void resample(noise::Particle& i_state);
    
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

protected:
    
    //! Motion Model
    CRMotionProbabilistic* m_motionModel;
    
    //! Sensor Model
    CRSensorProbabilistic* m_sensorModel;
    
    //! vector of particles
    std::vector<Eigen::VectorXd> m_particles;
    
    //! vector of weights
    std::vector<double> m_weights;
    
    //! critical number of particles - initially this is set to zero
    double m_Nresample = 0.0;
    
    //! Uniform random noise
    CRNoiseUniform* m_uniform;
    
};

} // namespace estimation
} // namespace cr

#endif
