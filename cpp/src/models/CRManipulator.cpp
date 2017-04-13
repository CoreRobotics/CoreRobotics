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

#include "CRManipulator.hpp"
#include <assert.h>


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor creates a manipulator.\n

 \param[in] i_type - the manipulator type (default: CR_MANIPULATOR_MODE_POSITION)
 */
//---------------------------------------------------------------------
CRManipulator::CRManipulator(CRManipulatorType i_type) {
	this->m_modelType = i_type;
	this->m_tipFrame = new CoreRobotics::CRFrame();
}

CRManipulator::CRManipulator() {
    this->m_modelType = CR_MANIPULATOR_MODE_POSITION;
	this->m_tipFrame = new CoreRobotics::CRFrame();
}
    
    
//=====================================================================
/*!
 This method sets the configuration space positions. If the robot is 
 fully defined by revolute joints, then this operation corresponds to
 setting the joint angle values.\n
 
 \param[in] i_q - vector of configuration values
 */
//---------------------------------------------------------------------
void CRManipulator::setConfiguration(Eigen::VectorXd i_q)
{
    for (size_t i = 0; i < m_listDriven.size(); i++) {
        m_listLinks.at(m_listDriven.at(i))->m_frame->setFreeValue(i_q(i));
    }
}
    
    
//=====================================================================
/*!
 This method gets the configuration space positions. If the robot is
 fully defined by revolute joints, then this operation corresponds to
 getting the joint angle values.\n
 
 \return - vector of configuration values
 */
//---------------------------------------------------------------------
Eigen::VectorXd CRManipulator::getConfiguration(void)
{
    Eigen::VectorXd q(m_listDriven.size());
    for (size_t i = 0; i < m_listDriven.size(); i++) {
        q(i) = m_listLinks.at(m_listDriven.at(i))->m_frame->getFreeValue();
    }
    return q;
}
    
    
//=====================================================================
/*!
 This method gets the forward kinematics (position of each frame) of 
 the manipulator.\n

 See: https://en.wikipedia.org/wiki/Forward_kinematics
 
 \return - matrix of forward kinematics (position of each frame in the manipulator)
 */
//---------------------------------------------------------------------
Eigen::MatrixXd CRManipulator::getForwardKinematics(void)
{
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::MatrixXd y(3,m_listParents.size()+1);
    y.col(0) = v;
    for (size_t k = 0; k < m_listParents.size(); k++) {
        int i = k;
        v << 0, 0, 0;
        while (i > -1) {
            v = m_listLinks.at(i)->m_frame->transformToParent(v);
            i = m_listParents.at(i);
        }
        y.col(k+1) = v;
    }
    return y;
}
    
    
//=====================================================================
/*!
 This method computes the full pose (position and orientation) numerical
 Jacobian of the manipulator for the current configuration with respect
 to the tool specified by the toolIndex.  See CRManipulator::addTool
 for adding tools to the manipulator.\n

 The size of the returned Jacobian is 6 x N where N is the number of
 free variables in the manipulator.  The rows of the Jacobian 
 correspond to the pose vector (x, y, z, a, b, g)^T.\n
 
 \param[in] i_toolIndex - index of the tool to be used to compute the Jacobian.
 \param[in] i_mode - the Euler convention to be used to specify the orientation.
 \param[in] i_poseElements - [optional] a boolean vector indicating which pose elements to return
 \return jacobian - (6 x N) jacobian matrix if poseElements is not specified OR
                        (M x N) jacobian for M true values in poseElements
 */
//---------------------------------------------------------------------
Eigen::MatrixXd CRManipulator::jacobian(unsigned i_toolIndex,
                                        CREulerMode i_mode)
{
    // Initialize the Jacobian matrix
    Eigen::MatrixXd J(6,m_listDriven.size());

	// pertubation size (see http://www.maths.lth.se/na/courses/FMN081/FMN081-06/lecture7.pdf)
    double delta = 1.0e-8;		// was 1.0e-9 - can improve this (adaptive?)

	// set up the variables
    Eigen::VectorXd q0;						// operating point
    Eigen::VectorXd qd;						// perturbed vector
    Eigen::Matrix<double, 6, 1> poseFwd;	// forward perturbation result
	Eigen::Matrix<double, 6, 1> poseBwd;	// backward perturbation result

	// intialize the configuration
    q0 = this->getConfiguration();

	// step through each driven variable
    for (size_t k = 0; k < m_listDriven.size(); k++) {

		// set the configuration operating point
        qd.setZero(q0.size(),1);

		// define which free variable is being perturbed
        qd(k) = delta;

		// perturb forward
        this->setConfiguration(q0+qd);
        poseFwd = this->getToolPose(i_toolIndex, i_mode);

		// perturb backward
        this->setConfiguration(q0-qd);
        poseBwd = this->getToolPose(i_toolIndex, i_mode);

		// central difference
		J.col(k) = (poseFwd - poseBwd) / (2.0*delta);
    }
    this->setConfiguration(q0);

	// zero out the m_tipFrame
	this->m_tipFrame->setRotationAndTranslation(Eigen::Matrix3d::Zero(),
                                                Eigen::Vector3d::Zero());
    
    // return the jacobian matrix
    return J;
}

    
// Numerical Jacobian overload (for reduced pose vector)
Eigen::MatrixXd CRManipulator::jacobian(unsigned i_toolIndex,
                                        CREulerMode i_mode,
                                        Eigen::Matrix<bool, 6, 1> i_poseElements)
{
    
    // Get the number of true elements in the pose vector & size the output
    int m = i_poseElements.cast<int>().sum();
    
    // Initialize the Jacobian matrix
    Eigen::MatrixXd J(m,m_listDriven.size());
    
    // pertubation size (see http://www.maths.lth.se/na/courses/FMN081/FMN081-06/lecture7.pdf)
    double delta = 1.0e-8;		// was 1.0e-9 - can improve this (adaptive?)
    
    // set up the variables
    Eigen::VectorXd q0;						// operating point
    Eigen::VectorXd qd;						// perturbed vector
    Eigen::VectorXd poseFwd;	// forward perturbation result
    Eigen::VectorXd poseBwd;	// backward perturbation result
    
    // intialize the configuration
    q0 = this->getConfiguration();
    
    // step through each driven variable
    for (size_t k = 0; k < m_listDriven.size(); k++) {
        
        // set the configuration operating point
        qd.setZero(q0.size(),1);
        
        // define which free variable is being perturbed
        qd(k) = delta;
        
        // perturb forward
        this->setConfiguration(q0+qd);
        poseFwd = this->getToolPose(i_toolIndex, i_mode, i_poseElements);
        
        // perturb backward
        this->setConfiguration(q0-qd);
        poseBwd = this->getToolPose(i_toolIndex, i_mode, i_poseElements);
        
        // central difference
        J.col(k) = (poseFwd - poseBwd) / (2.0*delta);
    }
    this->setConfiguration(q0);
    
    // zero out the m_tipFrame
    this->m_tipFrame->setRotationAndTranslation(Eigen::Matrix3d::Zero(),
                                                Eigen::Vector3d::Zero());
    
    // return the jacobian matrix
    return J;
    
}
    
    
//=====================================================================
/*!
 This method returns the number of rigid body links in the list. \n
 
 \return - number of rigid body links in the manipulator
 */
//---------------------------------------------------------------------
int CRManipulator::getNumberOfLinks(void)
{
    return (int)m_listParents.size();
}
    
    
//=====================================================================
/*!
 This method returns the number of driven rigid body links, i.e. the
 degrees of freedom (DOF). \n
 
 \return - degrees of freedom (DOF)
 */
//---------------------------------------------------------------------
int CRManipulator::getDegreesOfFreedom(void)
{
    return (int)m_listDriven.size();
}



//=====================================================================
/*!
This method returns a tool frame for the current robot pose.  Note that
a tool must have been added to the manipulator using the 
CRManipulator::addTool method prior to calling this method.\n

\param[in] i_toolIndex - index of the tool to query
\param[out] o_tool - the tool frame transformation referenced to the
robot base frame for the current manipulator configuration
*/
//---------------------------------------------------------------------
void CRManipulator::getToolFrame(unsigned i_toolIndex, CRFrame& o_tool)
{
	Eigen::Matrix4d T, T0;

	// return the transformation
	T = this->m_listToolFrames.at(i_toolIndex)->getTransformToParent();

	// now iterate back to the base frame
	int i = this->m_listToolParents.at(i_toolIndex);
	while (i > -1) {

		T0 = this->m_listLinks.at(i)->m_frame->getTransformToParent();
		T = T0*T;
		i = m_listParents.at(i);
	}
    
	o_tool.setRotationAndTranslation(T.block(0, 0, 3, 3), T.block(0, 3, 3, 1));
}
    
    

//=====================================================================
/*!
 This method returns a pose for the specified tool frame for the current
 robot pose.  Note that a tool must have been added to the manipulator 
 using the CRManipulator::addTool method prior to calling this method.\n
 
 \param[in] i_toolIndex - index of the tool to query
 \param[in] i_mode - the Euler convention for computing the pose
 \return - the pose for the specified tool
 robot base frame for the current manipulator configuration
 */
//---------------------------------------------------------------------
Eigen::Matrix<double, 6, 1> CRManipulator::getToolPose(unsigned i_toolIndex,
                                                       CREulerMode i_mode)
{
    // return the frame
    this->getToolFrame(i_toolIndex, *this->m_tipFrame);
    
    // get the pose of the frame
    return this->m_tipFrame->getPose(i_mode);
}
    
Eigen::VectorXd CRManipulator::getToolPose(unsigned i_toolIndex,
                                           CREulerMode i_mode,
                                           Eigen::Matrix<bool, 6, 1> i_poseElements)
{
    // return the frame
    this->getToolFrame(i_toolIndex, *this->m_tipFrame);
    
    // get the pose of the frame
    return this->m_tipFrame->getPose(i_mode, i_poseElements);
}

    
    
//=====================================================================
/*!
 This method adds a rigid body link the list of links. \n
 
 \param[in] i_link - pointer to the RigidBody link being added
 \return - returns the integer (index) of the added link
 */
//---------------------------------------------------------------------
int CRManipulator::addLink(CoreRobotics::CRRigidBody* i_link)
{
    int n = this->getNumberOfLinks();
    
    // add the link to the m_listLinks member
    m_listLinks.push_back(i_link);
    
    // add the parent integer
    m_listParents.push_back(n-1);
    
    // add the driven integer (if it is driven)
    if (i_link->m_frame->isDriven()) {
        m_listDriven.push_back(n);
    }
    
    // return the tool index
    return this->m_listLinks.size()-1;
}



//=====================================================================
/*!
This method adds a frame to the list of tools. \n

\param[in] parentIndex - the index of the parent frame to which the
supplied tool frame is referenced.
\param[in] tool - a CRFrame object containing the frame transformation 
to the specified parent link.
\return - returns the integer (index) of the added tool
*/
//---------------------------------------------------------------------
int CRManipulator::addTool(unsigned i_parentIndex, CRFrame* i_tool)
{
	// get the number of links
	int n = this->getNumberOfLinks();
	
    // throw an error if the parent index is outside of the range
    assert(i_parentIndex <= unsigned(n-1));
    
    // add the parent integer
    m_listToolFrames.push_back(i_tool);
    m_listToolParents.push_back(i_parentIndex);
    
    // return the tool index
    return this->m_listToolFrames.size()-1;
}


//=====================================================================
// End namespace
}
