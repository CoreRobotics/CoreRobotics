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
 
 \author CoreRobotics Project
 \author www.corerobotics.org
 \author Parker Owan
 \version 0.0
 
 */
//=====================================================================

#ifndef CRManipulator_hpp
#define CRManipulator_hpp

//=====================================================================
// Includes
#include <vector>
#include "../physics/CRRigidBody.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRManipulator.hpp
 \brief Implements a class that handles kinematics of a manipulator.
 */
//---------------------------------------------------------------------
/*!
 \class CRManipulator
 \ingroup models
 
 \brief This class implements a robotic manipulator composed of a
 series of rigid bodies.  Currently only supports serial manipulators.
 
 \details
 \section Description
 CRManipulator implements a robot manipulator as a series of rigid body
 links.  The free variables of the frames contained in the rigid bodies
 determine the degrees of freedom of the robot.
 
 These methods are used to build the manipulator:
 - CRManipulator::addLink adds a link to the kinematic chain.  Note that
 as of now, only serial manipulators are supported.
 - CRManipulator::mode is member that sets how the manipulator is
 driven.  Available options are in CoreRobotics::CRManipulatorType.
 
 These methods operate on the configuration space (i.e. joint space) 
 of the robot:
 - CRManipulator::getConfiguration outputs the vector of configuration
 values (i.e. joint angles for purely revolute joints).
 - CRManipulator::setConfiguration sets the configuration of the 
 manipulator.
 
 These methods return useful information for control:
 - CRManipulator::getForwardKinematics returns a matrix of the Euclidean
 poses at each frame (starting with the base frame and moving out to the
 tool frame) for the current configuration.
 - CRManipulator::getJacobian returns the tool space (last frame) 
 Jacobian matrix that maps joint velocities to tool velocities for the 
 current configuration.
 
 \section Example
 This example creates a CRManipulator class.
 \code
 
 #include "CoreRobotics.hpp"
 #include <stdio>
 
 const double CR_PI = 3.1415926535897932384626433832795;
 
 using namespace CoreRobotics;
 
 main() {
 
    CRManipulator MyRobot;

    // create a couple of rigid body links
    CRFrameEuler* F0 = new CRFrameEuler();
    CRFrameEuler* F1 = new CRFrameEuler();
    CRFrameEuler* F2 = new CRFrameEuler();
    CRRigidBody* Link0 = new CRRigidBody();
    CRRigidBody* Link1 = new CRRigidBody();
    CRRigidBody* Link2 = new CRRigidBody();


    // Set info for Link 0 and add to MyRobot
    F0->freeVar = CR_EULER_FREE_ANG_G;
    F0->setMode(CR_EULER_MODE_XYZ);
    F0->setPositionAndOrientation(0, 0, 0.5, 0, 0, 0);
    Link0->frame = F0;
    MyRobot.addLink(Link0);

    // Set info for Link 1 and add to MyRobot
    F1->freeVar = CR_EULER_FREE_ANG_G;
    F1->setMode(CR_EULER_MODE_XYZ);
    F1->setPositionAndOrientation(1, 0, 0, 0, 0, 0);
    Link1->frame = F1;
    MyRobot.addLink(Link1);

    // Set info for Link 2 and add to MyRobot
    F2->freeVar = CR_EULER_FREE_NONE;
    F2->setMode(CR_EULER_MODE_XYZ);
    F2->setPositionAndOrientation(2, 0, 0, 0, 0, 0);
    Link2->frame = F2;
    MyRobot.addLink(Link2);


    // Now get the configuration values
    int dof;
    Eigen::VectorXd jointAngles;
    MyRobot.getDegreesOfFreedom(dof);
    MyRobot.getConfiguration(jointAngles);
    std::cout << "MyRobot has " << dof << " DOF, with joint angles = ("
    << jointAngles.transpose() << ") rad" << std::endl;

    // Now get the Forward Kinematics and Jacobian
    Eigen::MatrixXd Jacobian, FwdKin;
    MyRobot.getForwardKinematics(FwdKin);
    MyRobot.getJacobian(Jacobian);
    std::cout << "Forward Kinematics = \n" << FwdKin << std::endl;
    std::cout << "Jacobian = \n" << Jacobian << std::endl;

    // now set a new robot configuration and get the FK and jacobian
    jointAngles << CR_PI/4.0, -CR_PI/4.0;
    std::cout << "Set joint angles = ("
    << jointAngles.transpose() << ") rad" << std::endl;
    MyRobot.setConfiguration(jointAngles);
    MyRobot.getForwardKinematics(FwdKin);
    MyRobot.getJacobian(Jacobian);
    std::cout << "Forward Kinematics = \n" << FwdKin << std::endl;
    std::cout << "Jacobian = \n" << Jacobian << std::endl;
 
 }
 
 \endcode
 
 \section References
 [1] J. Craig, "Introduction to Robotics: Mechanics and Control", Ed. 3,
 Pearson, 2004.
 
 [2] R. Murray, Z. Li, and S. Sastry, "A Mathematical Introduction to Robot
 Manipulation", Ed. 1, CRC Press, 1993.
 <a href="http://www.cds.caltech.edu/~murray/mlswiki/index.php/Main_Page">
 Available online.</a> \n\n
 */
//=====================================================================
//! Enumerator for specifying how the manipulator is driven
enum CRManipulatorType {
    CR_MANIPULATOR_MODE_POSITION,
    // CR_MANIPULATOR_MODE_VELOCITY,
    // CR_MANIPULATOR_MODE_TORQUE,
};
    
//=====================================================================
class CRManipulator {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
    CRManipulator();
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the configuration (joint) space positions
    void setConfiguration(Eigen::VectorXd q);
    
    //! Get the configuration (joint) space positions
    void getConfiguration(Eigen::VectorXd &q);
    
    //! Get the instantaneous forward kinematics
    void getForwardKinematics(Eigen::MatrixXd &y);
    
    //! Get the instantaneous numerical Jacobian
    void getJacobian(Eigen::MatrixXd &jacobian);
    
    //! Get the number of links in the list
    void getNumberOfLinks(int &n);
    
    //! Get the number of driven links (degrees of freedom: DOF)
    void getDegreesOfFreedom(int &dof);
    
//---------------------------------------------------------------------
// Public Methods
public:
    
    //! Add a link to the kinematic structure
    void addLink(CoreRobotics::CRRigidBody* link);
    
//---------------------------------------------------------------------
// Public Members
public:
    
    //! Define the manipulator type
    CRManipulatorType mode;
    
//---------------------------------------------------------------------
// Protected Members
protected:
    
    //! List of the links in the manipulator
    std::vector<CoreRobotics::CRRigidBody*> listLinks;
    
    //! List of the link parent indices
    std::vector<int> listParents;
    
    //! List of the driven link indices
    std::vector<int> listDriven;
    
};

//=====================================================================
// End namespace
}


#endif
