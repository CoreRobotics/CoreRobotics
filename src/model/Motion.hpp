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
//---------------------------------------------------------------------
// Begin header definition

#ifndef CR_MOTION_HPP_
#define CR_MOTION_HPP_

#include "Step.hpp"
#include "Eigen/Dense"

using namespace std::placeholders;

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace model {


//! Node shared pointer
class Motion;
typedef std::shared_ptr<Motion> MotionPtr;

//---------------------------------------------------------------------
/*!
 \class Motion
 \ingroup model
 
 \brief This class implements a motion model.
 
 \details
 ## Description
 MotionModel implements a motion model from a supplied dynamics
 callback function.  Specifically, MotionModel sets up a container
 for the continuous model
 
 \f[
 \dot{x} = f(t, x, u)
 \f]
 
 or
 
 \f[
 x_{k+1} = f(t_k, x_k, u_k)
 \f]
 
 where \f$x\f$ is the state vector, \f$u\f$ is the input vector,
 \f$t\f$ is time, and \f$k\f$ is a discrete sampling index.
 
 These methods are available for interfacing with the Motion Model:
 - MotionModel::setState sets the underlying state vector.
 - MotionModel::getState returns the state vector.
 - MotionModel::setTimeStep sets the time step (s).
 - MotionModel::getTimeStep returns the time step (s).
 - MotionModel::getTime returns the simulation time (s).
 - MotionModel::motion computes a new state and updates the
 internal value for an input (u).
 
 ## Example
 This example demonstrates use of the MotionModel class.
 \include example_MotionModel.cpp
 
 ## References
 [1] J. Crassidis and J. Junkins, "Optimal Estimation of Dynamic Systems",
 Ed. 2, CRC Press, 2012. \n\n
 
 [2] S. Thrun, W. Burgard, and D. Fox, "Probabilistic Robotics", MIT Press, 2006.
 \n\n
 */
//---------------------------------------------------------------------
class Motion
    : public core::Step
{

    // Constructor and Destructor
    public:
    
        //! Class constructor
        Motion(Eigen::VectorXd i_x0, Eigen::VectorXd i_u0, double m_dt);
    

    // Get/Set Methods
    public:
    
        //! Set the state vector (x)
        void setState(Eigen::VectorXd i_x) { m_state = i_x; }
    
        //! Get the state vector (x)
        Eigen::VectorXd getState() { return m_state; }
    
        //! Set the action vector (u)
        void setAction(Eigen::VectorXd i_u) { m_action = i_u; }
    
        //! Get the action vector (u)
        Eigen::VectorXd setAction() { return m_action; }
    
        //! Set the time step (s)
        void setTimeStep(double i_timeStep) { m_dt = i_timeStep; }
    
        //! Get the time step (s)
        double getTimeStep() { return m_dt; }
    
        //! Get the model time (s)
        double getTime() {return m_time;}
    
    
    // Step method
    public:
    
        //! step the motion (i.e. update the internal state)
        virtual void step();
    

    // Public Methods
    public:
    
        //! The prototype callback function \dot{x} = f(t, x, u) to be implemented
        virtual Eigen::VectorXd callback(double i_t,
                                         Eigen::VectorXd i_x,
                                         Eigen::VectorXd i_u) = 0;
    

    // Protected Members
    protected:
    
        //! bound callback function
        std::function<Eigen::VectorXd(double,Eigen::VectorXd,Eigen::VectorXd)> m_fcn = std::bind(&Motion::callback, this, _1, _2, _3);
    
        //! Sample rate (s)
        double m_dt;
    
        //! Current time (s)
        double m_time;
    
        //! Dynamic state of the system (x)
        Eigen::VectorXd m_state;
    
        //! Dynamic state of the system (u)
        Eigen::VectorXd m_action;
    
};


}
}
// end namespace
//---------------------------------------------------------------------


#endif

