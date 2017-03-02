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
 This method gets the numerical Jacobian of the manipulator for the 
 current configuration.\n
 
 \param[out] jacobian - jacobian matrix
 */
//---------------------------------------------------------------------
void CRManipulator::getJacobian(Eigen::MatrixXd &jacobian)
{
    double delta = 1.0e-9;
    Eigen::VectorXd q0;
    Eigen::VectorXd qd;
    Eigen::MatrixXd qFwd;
    Eigen::MatrixXd qBwd;
    jacobian.setZero(3,listDriven.size());
    this->getConfiguration(q0);
    for (size_t k = 0; k < listDriven.size(); k++) {
        qd.setZero(q0.size(),1);
        qd(k) = delta;
        this->setConfiguration(q0+qd);
        this->getForwardKinematics(qFwd);
        this->setConfiguration(q0-qd);
        this->getForwardKinematics(qBwd);
        jacobian.col(k) = (qFwd.rightCols(1)-qBwd.rightCols(1))/(2.0*delta);
    }
    this->setConfiguration(q0);
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
// End namespace
}
