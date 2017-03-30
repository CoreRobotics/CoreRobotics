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


//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
    
//=====================================================================
/*!
 The constructor creates a manipulator.\n
 */
//---------------------------------------------------------------------
CRManipulator::CRManipulator() {
    mode = CR_MANIPULATOR_MODE_POSITION;
	this->m_tipFrame = new CoreRobotics::CRFrame();
}
    
    
//=====================================================================
/*!
 This method sets the configuration space positions. If the robot is 
 fully defined by revolute joints, then this operation corresponds to
 setting the joint angle values.\n
 
 \param[in] q - configuration values
 */
//---------------------------------------------------------------------
void CRManipulator::setConfiguration(Eigen::VectorXd q)
{
    for (size_t i = 0; i < listDriven.size(); i++) {
        listLinks.at(listDriven.at(i))->frame->setFreeValue(q(i));
    }
}
    
    
//=====================================================================
/*!
 This method gets the configuration space positions. If the robot is
 fully defined by revolute joints, then this operation corresponds to
 getting the joint angle values.\n
 
 \param[out] q - configuration values
 */
//---------------------------------------------------------------------
void CRManipulator::getConfiguration(Eigen::VectorXd &q)
{
    q.setZero(listDriven.size(),1);
    for (size_t i = 0; i < listDriven.size(); i++) {
        listLinks.at(listDriven.at(i))->frame->getFreeValue(q(i));
    }
}
    
    
//=====================================================================
/*!
 This method gets the forward kinematics (position of each frame) of 
 the manipulator.\n

 See: https://en.wikipedia.org/wiki/Forward_kinematics
 
 \param[out] y - forward kinematics
 */
//---------------------------------------------------------------------
void CRManipulator::getForwardKinematics(Eigen::MatrixXd &y)
{
    Eigen::Vector3d v;
    y.setZero(3,listParents.size()+1);
    for (size_t k = 0; k < listParents.size(); k++) {
        int i = k;
        v << 0, 0, 0;
        while (i > -1) {
            listLinks.at(i)->frame->transformToParent(v,v);
            i = listParents.at(i);
        }
        y.col(k+1) = v;
    }
}
    
    
//=====================================================================
/*!
 This method gets the full pose (position and orientation) numerical 
 Jacobian of the manipulator for the current configuration with respect
 to the tool specified by the toolIndex.  See CRManipulator::addTool
 for adding tools to the manipulator.\n

 The size of the returned Jacobian is 6 x N where N is the number of
 free variables in the manipulator.  The rows of the Jacobian 
 correspond to the pose vector (x, y, z, a, b, g)^T.\n
 
 \param[in] toolIndex - index of the tool to be used to compute the Jacobian.
 \param[in] mode - the Euler convention to be used to specify the orientation.
 \param[out] jacobian - (6 x N) jacobian matrix.
 */
//---------------------------------------------------------------------
void CRManipulator::getJacobian(unsigned toolIndex, CREulerMode mode, Eigen::MatrixXd &jacobian)
{

	// pertubation size (see http://www.maths.lth.se/na/courses/FMN081/FMN081-06/lecture7.pdf)
    double delta = 1.0e-8;		// was 1.0e-9

	// set up the variables
    Eigen::VectorXd q0;						// operating point
    Eigen::VectorXd qd;						// perturbed vector
    Eigen::Matrix<double, 6, 1> poseFwd;	// forward perturbation result
	Eigen::Matrix<double, 6, 1> poseBwd;	// backward perturbation result

	// initialize the jacobian
    jacobian.setZero(6,listDriven.size());

	// intialize the configuration
    this->getConfiguration(q0);

	// step through each driven variable
    for (size_t k = 0; k < listDriven.size(); k++) {

		// set the configuration operating point
        qd.setZero(q0.size(),1);

		// define which free variable is being perturbed
        qd(k) = delta;

		// perturb forward
        this->setConfiguration(q0+qd);
		this->getToolFrame(toolIndex, *this->m_tipFrame);
		this->m_tipFrame->getPose(mode, poseFwd);
        // this->getForwardKinematics(fkFwd);

		// perturb backward
        this->setConfiguration(q0-qd);
		this->getToolFrame(toolIndex, *this->m_tipFrame);
		this->m_tipFrame->getPose(mode, poseBwd);
        // this->getForwardKinematics(fkBwd);

		// central difference
		jacobian.col(k) = (poseFwd - poseBwd) / (2.0*delta);
        // jacobian.col(k) = (fkFwd.rightCols(1)- fkBwd.rightCols(1))/(2.0*delta);
    }
    this->setConfiguration(q0);

	// zero out the m_tipFrame
	this->m_tipFrame->setRotationAndTranslation(Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero());
}

    
//=====================================================================
/*!
 This method returns the number of rigid body links in the list. \n
 
 \param[out] n - number of rigid body links in the manipulator
 */
//---------------------------------------------------------------------
void CRManipulator::getNumberOfLinks(int &n)
{
    n = (int)listParents.size();
}
    
    
//=====================================================================
/*!
 This method returns the number of driven rigid body links, i.e. the
 degrees of freedom (DOF). \n
 
 \param[out] dof - degrees of freedom (DOF)
 */
//---------------------------------------------------------------------
void CRManipulator::getDegreesOfFreedom(int &dof)
{
    dof = (int)listDriven.size();
}



//=====================================================================
/*!
This method returns a tool frame for the current robot pose.  Note that
a tool must have been added to the manipulator using the 
CRManipulator::addTool method prior to calling this method.\n

\param[in] toolIndex - index of the tool to query
\param[out] tool - the tool frame transformation referenced to the
robot base frame for the current manipulator configuration
*/
//---------------------------------------------------------------------
void CRManipulator::getToolFrame(unsigned toolIndex, CRFrame &tool)
{
	Eigen::Matrix4d T, T0;

	// return the transformation
	this->listToolFrames.at(toolIndex)->getTransformToParent(T);

	// now iterate back to the base frame
	int i = this->listToolParents.at(toolIndex);
	while (i > -1) {

		this->listLinks.at(i)->frame->getTransformToParent(T0);
		T = T0*T;
		i = listParents.at(i);
	}

	// tool.setRotationAndTranslation(rot, trans);
	tool.setRotationAndTranslation(T.block(0, 0, 3, 3), T.block(0, 3, 3, 1));
}

    
    
//=====================================================================
/*!
 This method adds a rigid body link the list of links. \n
 
 \param[in] link - pointer to the RigidBody link being added
 */
//---------------------------------------------------------------------
void CRManipulator::addLink(CoreRobotics::CRRigidBody* link)
{
    int n, dof;
    this->getNumberOfLinks(n);
    this->getDegreesOfFreedom(dof);
    
    // add the link to the listLinks member
    listLinks.push_back(link);
    
    // add the parent integer
    listParents.push_back(n-1);
    
    // add the driven integer (if it is driven)
    if (link->frame->isDriven()) {
        listDriven.push_back(n);
    }
}



//=====================================================================
/*!
This method adds a frame to the list of tools. \n

\param[in] parentIndex - the index of the parent frame to which the
supplied tool frame is referenced.
\param[in] tool - a CRFrame object containing the frame transformation 
to the specified parent link.
\return - returns true if the tool was set, returns a false if it was 
not set due to an invalid parentIndex
*/
//---------------------------------------------------------------------
bool CRManipulator::addTool(unsigned parentIndex, CRFrame* tool)
{
	// get the number of links
	int n;
	this->getNumberOfLinks(n);

	// add the parent integer
	if (parentIndex > unsigned(n-1)) {
		return false;
	} else {
		listToolFrames.push_back(tool);
		listToolParents.push_back(parentIndex);
		return true;
	}
}


//=====================================================================
// End namespace
}
