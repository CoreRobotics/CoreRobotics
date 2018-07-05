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

#include "Robot.hpp"
// #include <assert>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace world {
    
    
//---------------------------------------------------------------------
/*!
 The constructor creates a manipulator.\n
 */
//---------------------------------------------------------------------
Robot::Robot() {
    
}
    
    
//---------------------------------------------------------------------
/*!
 The destructor deletes a manipulator.\n
 */
//---------------------------------------------------------------------
Robot::~Robot() {
    
}
    
    
//---------------------------------------------------------------------
/*!
 Create a new manipulator in memory.\n
 */
//---------------------------------------------------------------------
RobotPtr Robot::create(){
    return std::make_shared<Robot>();
}
    
    
//---------------------------------------------------------------------
/*!
 Register a link with the Manipulator in order to have it computed in
 the Jacobian, Hessian, etc... methods.\n
 
 \param[in]     i_link - link to register with the Manipulator
 */
//---------------------------------------------------------------------
void Robot::addLink(LinkPtr i_link){
    m_links.push_back(i_link);
}
    
    
//---------------------------------------------------------------------
/*!
 This method returns the number of rigid body links in the list. \n
 
 \return    number of rigid body links in the manipulator
 */
//---------------------------------------------------------------------
int Robot::getNumberOfLinks()
{
    return (int)m_links.size();
}


//---------------------------------------------------------------------
/*!
 This method returns the number of driven rigid body links, i.e. the
 degrees of freedom (DOF). \n
 
 \return    degrees of freedom (DOF)
 */
//---------------------------------------------------------------------
int Robot::getDegreesOfFreedom()
{
    int n = 0;
    for (int i = 0; i < m_links.size(); i++) {
        if (m_links.at(i)->getLocalTransform().isDriven()) {
            n++;
        }
    }
    return n;
}
    
    
//=====================================================================
/*!
 This method sets the configuration space positions. If the robot is 
 fully defined by revolute joints, then this operation corresponds to
 setting the joint angle values.\n
 
 \param[in]     i_q         vector of configuration values
 */
//---------------------------------------------------------------------
    /*
void Robot::setConfiguration(Eigen::VectorXd i_q)
{
    for (size_t i = 0; i < m_listDriven.size(); i++) {
        m_listLinks.at(m_listDriven.at(i))->m_frame->setFreeValue(i_q(i));
    }
}
    */
     
    
//=====================================================================
/*!
 This method gets the configuration space positions. If the robot is
 fully defined by revolute joints, then this operation corresponds to
 getting the joint angle values.\n
 
 \return    vector of configuration values
 */
//---------------------------------------------------------------------
    /*
Eigen::VectorXd Robot::getConfiguration(void)
{
    Eigen::VectorXd q(m_listDriven.size());
    for (size_t i = 0; i < m_listDriven.size(); i++) {
        q(i) = m_listLinks.at(m_listDriven.at(i))->m_frame->getFreeValue();
    }
    return q;
}
     */
    
    
//=====================================================================
/*!
 This method gets the forward kinematics (position of each frame) of 
 the manipulator.\n

 See: https://en.wikipedia.org/wiki/Forward_kinematics
 
 \return    matrix of forward kinematics (position of each frame in the manipulator)
 */
//---------------------------------------------------------------------
    /*
Eigen::MatrixXd Robot::getForwardKinematics(void)
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
     */
    
    
//=====================================================================
/*!
 This method computes the full pose (position and orientation) numerical
 Jacobian of the manipulator for the current configuration with respect
 to the tool specified by the toolIndex.  See Robot::addTool
 for adding tools to the manipulator.\n

 The size of the returned Jacobian is 6 x N where N is the number of
 free variables in the manipulator.  The rows of the Jacobian 
 correspond to the pose vector (x, y, z, a, b, g)^T.\n
 
 \param[in]     i_toolIndex     index of the tool to be used to compute the Jacobian.
 \param[in]     i_mode          the Euler convention to be used to specify the orientation.
 \return                        (6 x N) jacobian matrix
 */
//---------------------------------------------------------------------
    /*
Eigen::MatrixXd Robot::jacobian(unsigned i_toolIndex,
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
     */


//=====================================================================
/*!
 This method computes the full pose (position and orientation) numerical
 Jacobian of the manipulator for the current configuration with respect
 to the tool specified by the toolIndex.  See Robot::addTool
 for adding tools to the manipulator.\n
 
 The size of the returned Jacobian is M x N where N is the number of
 free variables in the manipulator and M is the number of pose elements
 defined by true elements of i_poseElements.\n
 
 \param[in]     i_toolIndex     index of the tool to be used to compute the Jacobian.
 \param[in]     i_mode          the Euler convention to be used to specify the orientation.
 \param[in]     i_poseElements  a boolean vector indicating which pose elements to return
 \return                        (M x N) jacobian matrix for M true values in poseElements
 */
//---------------------------------------------------------------------
    /*
Eigen::MatrixXd Robot::jacobian(unsigned i_toolIndex,
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
     */

//=====================================================================
/*!
 This method computes the full pose (position and orientation) numerical
 Jacobian of the manipulator for the current configuration with respect
 to the tool specified by the toolIndex.  See Robot::addTool
 for adding tools to the manipulator.\n
 
 The size of the returned Jacobian is M x N where N is the number of
 free variables in the manipulator and M is the number of pose elements
 defined by nonzero elements of i_poseElementsInt.\n
 
 \param[in]     i_toolIndex     index of the tool to be used to compute the Jacobian.
 \param[in]     i_mode          the Euler convention to be used to specify the orientation.
 \param[in]     i_poseElementsInt  a integer vector indicating which pose elements to return
 \return                        (M x N) jacobian matrix for M true values in poseElements
 */
//---------------------------------------------------------------------
    /*
Eigen::MatrixXd Robot::jacobian(unsigned i_toolIndex,
                                        CREulerMode i_mode,
                                        Eigen::Matrix<int, 6, 1> i_poseElementsInt)
{
	Eigen::Matrix<bool, 6, 1> i_poseElements = i_poseElementsInt.cast<bool>();
	return Robot::jacobian(i_toolIndex, i_mode, i_poseElements);
}
     */
    

//=====================================================================
/*!
This method computes the full pose (position and orientation) numerical
Hessian of the manipulator for the current configuration with respect
to the tool specified by the toolIndex.  See Robot::addTool
for adding tools to the manipulator.\n

The size of the returned Hessian is 6 x N where N is the number of
free variables in the manipulator.  The rows of the Hessian
correspond to the pose vector (x, y, z, a, b, g)^T.\n

\param[in]     i_toolIndex     index of the tool to be used to compute the Hessian.
\param[in]     i_mode          the Euler convention to be used to specify the orientation.
\return                        (6 x N) Hessian matrix
*/
//---------------------------------------------------------------------
    /*
Eigen::MatrixXd Robot::hessian(unsigned i_toolIndex,
										CREulerMode i_mode)
{
	// Initialize the Hessian matrix
	Eigen::MatrixXd H(6, m_listDriven.size());

	// pertubation size (see http://www.maths.lth.se/na/courses/FMN081/FMN081-06/lecture7.pdf)
	double delta = 1.0e-6;		// was 1.0e-9 - can improve this (adaptive?)

								// set up the variables
	Eigen::VectorXd q0;						// operating point
	Eigen::VectorXd qd;						// perturbed vector
	Eigen::Matrix<double, 6, 1> poseFwd;	// forward perturbation result
	Eigen::Matrix<double, 6, 1> poseNow;	// current location result
	Eigen::Matrix<double, 6, 1> poseBwd;	// backward perturbation result

											// intialize the configuration
	q0 = this->getConfiguration();

	// step through each driven variable
	for (size_t k = 0; k < m_listDriven.size(); k++) {

		// set the configuration operating point
		qd.setZero(q0.size(), 1);

		// define which free variable is being perturbed
		qd(k) = delta;

		// perturb forward
		this->setConfiguration(q0 + qd*2.0);
		poseFwd = this->getToolPose(i_toolIndex, i_mode);

		// get current
		this->setConfiguration(q0);
		poseNow = this->getToolPose(i_toolIndex, i_mode);

		// perturb backward
		this->setConfiguration(q0 - qd*2.0);
		poseBwd = this->getToolPose(i_toolIndex, i_mode);

		// central difference
		H.col(k) = (poseFwd + poseBwd - 2.0*poseNow) / (4.0*delta*delta);
	}
	this->setConfiguration(q0);

	// zero out the m_tipFrame
	this->m_tipFrame->setRotationAndTranslation(Eigen::Matrix3d::Zero(),
		Eigen::Vector3d::Zero());

	// return the hessian matrix
	return H;
}
     */


//=====================================================================
/*!
This method computes the full pose (position and orientation) numerical
Hessian of the manipulator for the current configuration with respect
to the tool specified by the toolIndex.  See Robot::addTool
for adding tools to the manipulator.\n

The size of the returned Hessian is M x N where N is the number of
free variables in the manipulator and M is the number of pose elements
defined by true elements of i_poseElements.\n

\param[in]     i_toolIndex     index of the tool to be used to compute the hessian.
\param[in]     i_mode          the Euler convention to be used to specify the orientation.
\param[in]     i_poseElements  a boolean vector indicating which pose elements to return
\return                        (M x N) hessian matrix for M true values in poseElements
*/
//---------------------------------------------------------------------
    /*
Eigen::MatrixXd Robot::hessian(unsigned i_toolIndex,
										CREulerMode i_mode,
										Eigen::Matrix<bool, 6, 1> i_poseElements)
{

	// Get the number of true elements in the pose vector & size the output
	int m = i_poseElements.cast<int>().sum();

	// Initialize the hessian matrix
	Eigen::MatrixXd H(m, m_listDriven.size());

	// pertubation size (see http://www.maths.lth.se/na/courses/FMN081/FMN081-06/lecture7.pdf)
	double delta = 1.0e-6;		// was 1.0e-9 - can improve this (adaptive?)

								// set up the variables
	Eigen::VectorXd q0;						// operating point
	Eigen::VectorXd qd;						// perturbed vector
	Eigen::VectorXd poseFwd;	// forward perturbation result
	Eigen::VectorXd poseNow;	// current location result
	Eigen::VectorXd poseBwd;	// backward perturbation result

								// intialize the configuration
	q0 = this->getConfiguration();

	// step through each driven variable
	for (size_t k = 0; k < m_listDriven.size(); k++) {

		// set the configuration operating point
		qd.setZero(q0.size(), 1);

		// define which free variable is being perturbed
		qd(k) = delta;

		// perturb forward
		this->setConfiguration(q0 + 2.0*qd);
		poseFwd = this->getToolPose(i_toolIndex, i_mode, i_poseElements);

		// current pose
		this->setConfiguration(q0);
		poseNow = this->getToolPose(i_toolIndex, i_mode, i_poseElements);

		// perturb backward
		this->setConfiguration(q0 - 2.0*qd);
		poseBwd = this->getToolPose(i_toolIndex, i_mode, i_poseElements);

		// central difference
		H.col(k) = (poseFwd + poseBwd - 2.0*poseNow) / (4.0*delta*delta);
	}
	this->setConfiguration(q0);

	// zero out the m_tipFrame
	this->m_tipFrame->setRotationAndTranslation(Eigen::Matrix3d::Zero(),
		Eigen::Vector3d::Zero());

	// return the hessian matrix
	return H;

}
     */

//=====================================================================
/*!
This method computes the full pose (position and orientation) numerical
Hessian of the manipulator for the current configuration with respect
to the tool specified by the toolIndex.  See Robot::addTool
for adding tools to the manipulator.\n

The size of the returned Hessian is M x N where N is the number of
free variables in the manipulator and M is the number of pose elements
defined by nonzero elements of i_poseElementsInt.\n

\param[in]     i_toolIndex     index of the tool to be used to compute the Hessian.
\param[in]     i_mode          the Euler convention to be used to specify the orientation.
\param[in]     i_poseElementsInt  a integer vector indicating which pose elements to return
\return                        (M x N) hessian matrix for M true values in poseElements
*/
//---------------------------------------------------------------------
    /*
Eigen::MatrixXd Robot::hessian(unsigned i_toolIndex,
	CREulerMode i_mode,
	Eigen::Matrix<int, 6, 1> i_poseElementsInt)
{
	Eigen::Matrix<bool, 6, 1> i_poseElements = i_poseElementsInt.cast<bool>();
	return Robot::hessian(i_toolIndex, i_mode, i_poseElements);
}
     */


//=====================================================================
/*!
This method returns a tool frame for the current robot pose.  Note that
a tool must have been added to the manipulator using the 
Robot::addTool method prior to calling this method.\n

\param[in]      i_toolIndex     index of the tool to query
\param[out]     o_tool          the tool frame transformation referenced
                                to the robot base frame for the current
                                manipulator configuration
*/
//---------------------------------------------------------------------
    /*
void Robot::getToolFrame(unsigned i_toolIndex, Frame& o_tool)
{
	Eigen::Matrix4d T, T0;

	// return the transformation
	T = this->m_listToolFrames.at(i_toolIndex)->getTransformToParent();

	// now iterate back to the base frame
	int i = this->m_listToolParents.at(i_toolIndex);
	while (i > -1) {

		T0 = this->m_listLinks.at(i)->m_frame->getTransformToParent();
		T = T0*T;
		i = this->m_listParents.at(i);
	}
    
	o_tool.setRotationAndTranslation(T.block(0, 0, 3, 3), T.block(0, 3, 3, 1));
}
    */



//=====================================================================
/*!
This method returns a link frame for the current robot pose.  Note that
a link must have been added to the manipulator using the
Robot::addLink method prior to calling this method.\n

\param[in]      i_linkIndex     index of the link to query
\param[out]     o_link          the link frame transformation referenced
to the robot base frame for the current manipulator configuration
*/
//---------------------------------------------------------------------
    /*
void Robot::getLinkFrame(unsigned i_linkIndex, Frame& o_link)
{
	Eigen::Matrix4d T, T0;

	// return the transformation
	// T = this->m_listLinks.at(i_linkIndex)->m_frame->getTransformToParent();
	T.setIdentity();

	// now iterate back to the base frame
	// int i = this->m_listParents.at(i_linkIndex);
	int i = i_linkIndex;
	while (i > -1) {

		T0 = this->m_listLinks.at(i)->m_frame->getTransformToParent();
		T = T0*T;
		i = this->m_listParents.at(i);
	}

	o_link.setRotationAndTranslation(T.block(0, 0, 3, 3), T.block(0, 3, 3, 1));
}
    */
    
    

//=====================================================================
/*!
 This method returns a pose for the specified tool frame for the current
 robot pose.  Note that a tool must have been added to the manipulator 
 using the Robot::addTool method prior to calling this method.\n
 
 \param[in]     i_toolIndex     index of the tool to query
 \param[in]     i_mode          the Euler convention for computing the pose
 \return                        the pose for the specified tool
                                robot base frame for the current 
                                manipulator configuration
 */
//---------------------------------------------------------------------
    /*
Eigen::Matrix<double, 6, 1> Robot::getToolPose(unsigned i_toolIndex,
                                                       CREulerMode i_mode)
{
    // return the frame
    this->getToolFrame(i_toolIndex, *this->m_tipFrame);
    
    // get the pose of the frame
    return this->m_tipFrame->getPose(i_mode);
}
    
Eigen::VectorXd Robot::getToolPose(unsigned i_toolIndex,
                                           CREulerMode i_mode,
                                           Eigen::Matrix<bool, 6, 1> i_poseElements)
{
    // return the frame
    this->getToolFrame(i_toolIndex, *this->m_tipFrame);
    
    // get the pose of the frame
    return this->m_tipFrame->getPose(i_mode, i_poseElements);
}

Eigen::VectorXd Robot::getToolPose(unsigned i_toolIndex,
                                           CREulerMode i_mode,
                                           Eigen::Matrix<int, 6, 1> i_poseElementsInt)
{
	Eigen::Matrix<bool, 6, 1> i_poseElements = i_poseElementsInt.cast<bool>();
	return Robot::getToolPose(i_toolIndex, i_mode, i_poseElements);
}
    */
    
    
//=====================================================================
/*!
 This method adds a rigid body link the list of links. \n
 
 \param[in]     i_link          pointer to the RigidBody link being added
 \return                        returns the integer (index) of the added link
 */
//---------------------------------------------------------------------
    /*
int Robot::addLink(cr::RigidBody* i_link)
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
    */



//=====================================================================
/*!
This method adds a frame to the list of tools. \n

\param[in]      i_parentIndex       the index of the parent frame to which the
                                    supplied tool frame is referenced.
\param[in]      i_tool              a Frame object containing the 
                                    frame transformation to the specified
                                    parent link.
\return                             returns the integer (index) of the added tool
*/
//---------------------------------------------------------------------
    /*
int Robot::addTool(unsigned i_parentIndex, Frame* i_tool)
{
	// get the number of links
	// int n = this->getNumberOfLinks();
	
    // throw an error if the parent index is outside of the range
    assert(i_parentIndex <= unsigned(this->getNumberOfLinks()-1));
    
    // add the parent integer
    m_listToolFrames.push_back(i_tool);
    m_listToolParents.push_back(i_parentIndex);
    
    // return the tool index
    return this->m_listToolFrames.size()-1;
}
     */
    
    
//---------------------------------------------------------------------
/*!
 Print the children.\n
 */
//---------------------------------------------------------------------
void Robot::print(std::ostream& i_stream)
{
    unsigned d = getDepth();
    for (int i = 0; i < d; i++){
        i_stream << "  ";
    }
    std::string id = "N";
    if (isLeaf()) {
        id = "L";
    } else if (isRoot()) {
        id = "R";
    }
    i_stream << "+ cr::Manipulator [" << id << "] DOF = " <<
        getDegreesOfFreedom() << " '" << getName() << "'\n";
    for (int i = 0; i < m_children.size(); i++)
    {
        m_children.at(i)->print(i_stream);
    }
}


}
}
// end namespace
//---------------------------------------------------------------------
