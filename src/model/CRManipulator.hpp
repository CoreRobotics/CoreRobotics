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

#ifndef CRManipulator_hpp
#define CRManipulator_hpp

//=====================================================================
// Includes
#include "core/CRTypes.hpp"
#include <vector>
#include "physics/CRRigidBody.hpp"

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics  {
    
//=====================================================================
/*!
 \file CRManipulator.hpp
 \brief Implements a class that handles kinematics of a manipulator.
 */
//---------------------------------------------------------------------
/*!
 \class CRManipulator
 \ingroup model
 
 \brief This class implements a robotic manipulator composed of a
 series of rigid bodies.  Currently only supports serial manipulators.
 
 \details
 ## Description
 CRManipulator implements a robot manipulator as a series of rigid body
 links.  The free variables of the frames contained in the rigid bodies
 determine the degrees of freedom of the robot.
 
 These methods are used to build the manipulator:
 - CRManipulator::addLink adds a link to the kinematic chain.  Note that
 as of now, only serial manipulators are supported.
 - CRManipulator::addTool adds a tool to the chain relative to a link
 frame in the kinematic chain
 - CRManipulator::setModelType sets how the manipulator is driven.
 Available options are defined in CoreRobotics::CRManipulatorType.
 
 These methods operate on the configuration space (i.e. joint space) 
 of the robot:
 - CRManipulator::getConfiguration outputs the vector of configuration
 values (i.e. joint angles for purely revolute joints).
 - CRManipulator::setConfiguration sets the configuration of the
 manipulator.
 - CRManipulator::getLinkFrame returns the specified link frame for the
 current manipulator configuration.
 - CRManipulator::getToolFrame returns the specified tool frame for the
 current manipulator configuration.
 - CRManipulator::getToolPose returns the pose of the specified tool.
 - CRManipulator::getDegreesOfFreedom returns the number of free
 configuration variables.
 - CRManipulator::getNumberOfLinks returns the number of frames in the
 tree.
 
 These methods return useful information for control:
 - CRManipulator::getForwardKinematics returns a matrix of the Euclidean
 poses at each frame (starting with the base frame and moving out to the
 tool frame) for the current configuration.
 - CRManipulator::jacobian returns the Jacobian for a specified tool.
 
 ## Example
 This example shows use of CRManipulator.

 \include example_CRManipulator.cpp
 
 ## References
 [1] J. Craig, "Introduction to Robotics: Mechanics and Control", Ed. 3,
 Pearson, 2004.
 
 [2] R. Murray, Z. Li, and S. Sastry, "A Mathematical Introduction to Robot
 Manipulation", Ed. 1, CRC Press, 1993.
 <a href="http://www.cds.caltech.edu/~murray/mlswiki/index.php/Main_Page">
 Available online.</a> \n\n

 [3] A. Hourtash, "The Kinematic Hessian and Higher Derivatives", IEEE 
 International Symposium on Computational Intelligence in Robotics and 
 Automation, 169-174, 2005. 
 <a href="http://doi.org/10.1109/CIRA.2005.1554272">

 */
//=====================================================================
//! Enumerator for specifying how the manipulator is driven
enum [[deprecated(CR_DEPRECATED)]] CRManipulatorType {
	// TODO: add several modes of control for the driving values
    CR_MANIPULATOR_MODE_POSITION,
    // CR_MANIPULATOR_MODE_VELOCITY,
    // CR_MANIPULATOR_MODE_TORQUE,
};
    
//=====================================================================
class [[deprecated(CR_DEPRECATED)]] CRManipulator {
    
//---------------------------------------------------------------------
// Constructor and Destructor
public:
    
    //! Class constructor
	CRManipulator(CRManipulatorType i_type);
    CRManipulator();
    
//---------------------------------------------------------------------
// Get/Set Methods
public:
    
    //! Set the configuration (joint) space positions
    void setConfiguration(Eigen::VectorXd i_q);
    
    //! Get the configuration (joint) space positions
    Eigen::VectorXd getConfiguration(void);
    
    //! Get the instantaneous forward kinematics
    Eigen::MatrixXd getForwardKinematics(void);
    
    //! Get the number of links in the list
    int getNumberOfLinks(void);
    
    //! Get the number of driven links (degrees of freedom: DOF)
    int getDegreesOfFreedom(void);

	//! Get tool frame for the current manipulator configuration
	void getToolFrame(unsigned i_toolIndex, CRFrame& o_tool);

	//! Get a link frame for the current manipulator configuration
	void getLinkFrame(unsigned i_linkIndex, CRFrame& o_link);
    
    //! Get the pose for the specified tool index
    Eigen::Matrix<double, 6, 1> getToolPose(unsigned i_toolIndex,
                                            CREulerMode i_mode);
    
    Eigen::VectorXd getToolPose(unsigned i_toolIndex,
                                CREulerMode i_mode,
                                Eigen::Matrix<bool, 6, 1> i_poseElements);

	Eigen::VectorXd getToolPose(unsigned i_toolIndex,
                                CREulerMode i_mode,
                                Eigen::Matrix<int, 6, 1> i_poseElementsInt);

	//! Set the model type
	void setModelType(CRManipulatorType type) { this->m_modelType = type; }
    
//---------------------------------------------------------------------
// Jacobian
public:
    
    //! Compute the instantaneous numerical Jacobian
    Eigen::MatrixXd jacobian(unsigned i_toolIndex,
                             CREulerMode i_mode);
    
    Eigen::MatrixXd jacobian(unsigned i_toolIndex,
                             CREulerMode i_mode,
                             Eigen::Matrix<bool, 6, 1> i_poseElements);

	Eigen::MatrixXd jacobian(unsigned i_toolIndex,
                             CREulerMode i_mode,
                             Eigen::Matrix<int, 6, 1> i_poseElementsInt);

//---------------------------------------------------------------------
// Hessian
public:

	//! Compute the instantaneous numerical Hessian
	Eigen::MatrixXd hessian(unsigned i_toolIndex,
		CREulerMode i_mode);

	Eigen::MatrixXd hessian(unsigned i_toolIndex,
		CREulerMode i_mode,
		Eigen::Matrix<bool, 6, 1> i_poseElements);

	Eigen::MatrixXd hessian(unsigned i_toolIndex,
		CREulerMode i_mode,
		Eigen::Matrix<int, 6, 1> i_poseElementsInt);


//---------------------------------------------------------------------
// Add link/tool Methods
public:
    
    //! Add a link to the kinematic structure, return the index of the added link
    int addLink(CoreRobotics::CRRigidBody* i_link);

	//! Add a tool to the manipulator, return the index of the added tool
	int addTool(unsigned i_parentIndex, CRFrame* i_tool);
    
//---------------------------------------------------------------------
// Protected Members
protected:

	//! Define the manipulator type specifying the dynamics model
	CRManipulatorType m_modelType;
    
    //! List of the links in the manipulator
    std::vector<CoreRobotics::CRRigidBody*> m_listLinks;
    
    //! List of the link parent indices
    std::vector<int> m_listParents;
    
    //! List of the driven link indices
    std::vector<int> m_listDriven;

	//! List of tool frames
	std::vector<CoreRobotics::CRFrame*> m_listToolFrames;

	//! List of tool parents
	std::vector<int> m_listToolParents;

	//! Arbitrary frame for computing tip position
	CoreRobotics::CRFrame* m_tipFrame;
    
};

//=====================================================================
// End namespace
}


#endif
