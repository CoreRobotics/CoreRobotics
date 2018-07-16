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
#include <iostream>
#include "CoreRobotics.hpp"
#include "gtest/gtest.h"

// Use the CoreRobotics namespace
using namespace cr;


//
// Setup robot convenience function
//
void setupRobot(cr::world::RobotPtr myRobot, cr::world::LinkPtr myLink[3])
{
    // Define an inertia tensor matrix and com vector
    Eigen::Matrix3d I;
    Eigen::Vector3d COM;
    
    // Create a rigid body
    cr::RigidBody rb;
    
    // Create a frame for writing the data
    cr::FrameEuler fe;
    fe.setMode(cr::CR_EULER_MODE_XYZ);
    
    fe.setPositionAndOrientation(0, 0, 1, 0, 0, M_PI / 3.0);
    I << 0, 0, 0, 0, 0, 0, 0, 0, 0.1;
    COM << 0.25, 0, 0;
    rb.setMass(1.1);
    rb.setInertiaTensor(I);
    rb.setCenterOfMass(COM);
    myLink[0] = cr::world::Link::create();
    myLink[0]->setLocalTransform(fe);
    myLink[0]->setRigidBody(rb);
    myLink[0]->setDegreeOfFreedom(cr::CR_EULER_FREE_ANG_G);
    myRobot->addChild(myLink[0]);
    
    fe.setPositionAndOrientation(1, 0, 0, 0, 0, -M_PI / 4.0);
    I << 0, 0, 0, 0, 0, 0, 0, 0, 0.2;
    COM << 0.6, 0, 0;
    rb.setMass(1.8);
    rb.setInertiaTensor(I);
    rb.setCenterOfMass(COM);
    myLink[1] = cr::world::Link::create();
    myLink[1]->setLocalTransform(fe);
    myLink[1]->setRigidBody(rb);
    myLink[1]->setDegreeOfFreedom(cr::CR_EULER_FREE_ANG_G);
    myLink[0]->addChild(myLink[1]);
    
    fe.setPositionAndOrientation(2, 0, 0, 0, 0, 0);
    myLink[2] = cr::world::Link::create();
    myLink[2]->setLocalTransform(fe);
    myLink[2]->setDegreeOfFreedom(cr::CR_EULER_FREE_NONE);
    myLink[1]->addChild(myLink[2]);
    
    // set the links
    myRobot->addLink(myLink[0]);
    myRobot->addLink(myLink[1]);
    myRobot->addLink(myLink[2]);
}



//
// setup the manipulator
//
TEST(Manipulator, Setup){
    
    // setup the world and robot
    cr::world::OriginPtr simWorld = cr::world::Origin::create();
    cr::world::RobotPtr robot = cr::world::Robot::create();
    simWorld->addChild(robot);
    
    // Setup the rigid body link items
    cr::world::LinkPtr link[3];
    
    // initialize the robot
    setupRobot(robot, link);
    
    // make sure the scene graph worked
    EXPECT_EQ(link[0]->getParent(), robot);
    EXPECT_EQ(link[1]->getParent(), link[0]);
    EXPECT_EQ(link[2]->getParent(), link[1]);
    
    // check degrees of freedom and links
    EXPECT_EQ(robot->getNumberOfLinks(), 3);
    EXPECT_EQ(robot->getDegreesOfFreedom(), 2);
    
    // check configuration
    Eigen::VectorXd q;
    q = robot->getConfiguration();
    EXPECT_EQ(q.size(), 2);
    EXPECT_DOUBLE_EQ(q(0), M_PI / 3.0);
    EXPECT_DOUBLE_EQ(q(1), -M_PI / 4.0);
}


//
// setup the manipulator
//
TEST(Manipulator, Kinematics){
    
    // setup the world and robot
    cr::world::OriginPtr simWorld = cr::world::Origin::create();
    cr::world::RobotPtr robot = cr::world::Robot::create();
    simWorld->addChild(robot);
    
    // Setup the rigid body link items
    cr::world::LinkPtr link[3];
    
    // initialize the robot
    setupRobot(robot, link);
    
    // get the configuration
    Eigen::VectorXd q = robot->getConfiguration();
    
    // get the global positions
    Eigen::Matrix<double, 6, 1> p;
    p = link[0]->getGlobalTransform().getPose(CR_EULER_MODE_XYZ);
    EXPECT_DOUBLE_EQ(p(0), 0);
    EXPECT_DOUBLE_EQ(p(1), 0);
    EXPECT_DOUBLE_EQ(p(2), 1);
    EXPECT_DOUBLE_EQ(p(3), 0);
    EXPECT_DOUBLE_EQ(p(4), 0);
    EXPECT_DOUBLE_EQ(p(5), q(0));
    
    p = link[1]->getGlobalTransform().getPose(CR_EULER_MODE_XYZ);
    EXPECT_DOUBLE_EQ(p(0), 1 * cos ( q(0) ) );
    EXPECT_DOUBLE_EQ(p(1), 1 * sin ( q(0) ) );
    EXPECT_DOUBLE_EQ(p(2), 1);
    EXPECT_DOUBLE_EQ(p(3), 0);
    EXPECT_DOUBLE_EQ(p(4), 0);
    EXPECT_DOUBLE_EQ(p(5), q(0) + q(1));
    
    p = link[2]->getGlobalTransform().getPose(CR_EULER_MODE_XYZ);
    EXPECT_DOUBLE_EQ(p(0), 2 * cos ( q(0) + q(1) ) + 1 * cos ( q(0) ));
    EXPECT_DOUBLE_EQ(p(1), 2 * sin ( q(0) + q(1) ) + 1 * sin ( q(0) ));
    EXPECT_DOUBLE_EQ(p(2), 1);
    EXPECT_DOUBLE_EQ(p(3), 0);
    EXPECT_DOUBLE_EQ(p(4), 0);
    EXPECT_DOUBLE_EQ(p(5), q(0) + q(1));

}


//
// Get Jacobian
//
TEST(Manipulator, Jacobian){
    
    // setup the world and robot
    cr::world::OriginPtr simWorld = cr::world::Origin::create();
    cr::world::RobotPtr robot = cr::world::Robot::create();
    simWorld->addChild(robot);
    
    // Setup the rigid body link items
    cr::world::LinkPtr link[3];
    
    // initialize the robot
    setupRobot(robot, link);
    
    // get the configuration
    Eigen::VectorXd q = robot->getConfiguration();
    
    // compute the jacobian
    Eigen::MatrixXd J;
    J = robot->jacobian(link[2], cr::CR_EULER_MODE_XYZ);
    EXPECT_EQ(6, J.rows());
    EXPECT_EQ(2, J.cols());
    
    // test the jacobian (to known exact) - we use near 1e-6 small angle theorem
    // due to the numerical centeral difference method for the computation
    EXPECT_NEAR(J(0, 0), - 1 * sin ( q(0) ) - 2 * sin ( q(0) + q(1) ), 1e-6);
    EXPECT_NEAR(J(0, 1), - 2 * sin( q(0) + q(1) ), 1e-6);
    EXPECT_NEAR(J(1, 0), + 1 * cos ( q(0) ) + 2 * cos ( q(0) + q(1) ), 1e-6);
    EXPECT_NEAR(J(1, 1), + 2 * cos ( q(0) + q(1) ), 1e-6);
    EXPECT_NEAR(J(2, 0), 0, 1e-6);
    EXPECT_NEAR(J(2, 1), 0, 1e-6);
    EXPECT_NEAR(J(3, 0), 0, 1e-6);
    EXPECT_NEAR(J(3, 1), 0, 1e-6);
    EXPECT_NEAR(J(4, 0), 0, 1e-6);
    EXPECT_NEAR(J(4, 1), 0, 1e-6);
    EXPECT_NEAR(J(5, 0), 1, 1e-6);
    EXPECT_NEAR(J(5, 1), 1, 1e-6);
}


//
// Get Mass
//
TEST(Manipulator, Mass){
    
    // setup the world and robot
    cr::world::OriginPtr simWorld = cr::world::Origin::create();
    cr::world::RobotPtr robot = cr::world::Robot::create();
    simWorld->addChild(robot);
    
    // Setup the rigid body link items
    cr::world::LinkPtr link[3];
    
    // initialize the robot
    setupRobot(robot, link);
    
    // get the configuration
    Eigen::VectorXd q = robot->getConfiguration();
    
    // compute the mass matrix
    Eigen::MatrixXd M;
    M = robot->mass();
    EXPECT_EQ(2, M.rows());
    EXPECT_EQ(2, M.cols());
    
    // compute the analytic intermediate parameters
    double I1 = 0.1;
    double I2 = 0.2;
    double m1 = 1.1;
    double m2 = 1.8;
    double r1 = 0.25;
    double r2 = 0.6;
    double l1 = 1.0;
    double a = I1 + I2 + m1*r1*r1 + m2*(l1*l1 + r2*r2);
    double b = m2*l1*r2;
    double d = I2 + m2*r2*r2;
    
    // test to the analytic solution
    EXPECT_NEAR(M(0, 0), a + 2 * b * cos ( q(1) ), 1e-6);
    EXPECT_NEAR(M(0, 1), d + b * cos ( q(1) ), 1e-6);
    EXPECT_NEAR(M(1, 0), d + b * cos ( q(1) ), 1e-6);
    EXPECT_NEAR(M(1, 1), d, 1e-6);
}


//
// Get Hessian
//
/*
TEST(Manipulator, Hessian){
    world::Manipulator MyRobot;
    int toolIndex = setupRobot(MyRobot);
    
    // NOTE: the numerical hessian is noisy!
    Eigen::MatrixXd H;
    H = MyRobot.hessian(toolIndex, CR_EULER_MODE_XYZ);
    EXPECT_EQ(6, H.rows());
    EXPECT_EQ(2, H.cols());
    EXPECT_NEAR(-3, H(0, 0), 1e-3);
    EXPECT_DOUBLE_EQ(0, H(1, 0));
    EXPECT_DOUBLE_EQ(0, H(2, 0));
    EXPECT_DOUBLE_EQ(0, H(3, 0));
    EXPECT_DOUBLE_EQ(0, H(4, 0));
    EXPECT_DOUBLE_EQ(0, H(5, 0));
    EXPECT_NEAR(-2, H(0, 1), 1e-3);
    EXPECT_DOUBLE_EQ(0, H(1, 1));
    EXPECT_DOUBLE_EQ(0, H(2, 1));
    EXPECT_DOUBLE_EQ(0, H(3, 1));
    EXPECT_DOUBLE_EQ(0, H(4, 1));
    EXPECT_DOUBLE_EQ(0, H(5, 1));
    
    Eigen::Matrix<bool, 6, 1> pe;
    pe << true, true, false, false, false, true;
    H = MyRobot.hessian(toolIndex, CR_EULER_MODE_XYZ, pe);
    EXPECT_EQ(3, H.rows());
    EXPECT_EQ(2, H.cols());
    EXPECT_NEAR(-3, H(0, 0), 1e-3);
    EXPECT_DOUBLE_EQ(0, H(1, 0));
    EXPECT_DOUBLE_EQ(0, H(2, 0));
    EXPECT_NEAR(-2, H(0, 1), 1e-3);
    EXPECT_DOUBLE_EQ(0, H(1, 1));
    EXPECT_DOUBLE_EQ(0, H(2, 1));
}
 */



//
// Get tool/link frames
//
/*
TEST(Manipulator, GetFrames){
    world::Manipulator MyRobot;
    int toolIndex = setupRobot(MyRobot);
    
    // now get the transformation to the tool for the current configuration
    Eigen::Matrix4d T;
    Frame toolFrame;
    MyRobot.getToolFrame(toolIndex, toolFrame);
    T = toolFrame.getTransformToParent();
    EXPECT_DOUBLE_EQ(1, T(0, 0));
    EXPECT_DOUBLE_EQ(0, T(0, 1));
    EXPECT_DOUBLE_EQ(0, T(0, 2));
    EXPECT_DOUBLE_EQ(3, T(0, 3));
    EXPECT_DOUBLE_EQ(0, T(1, 0));
    EXPECT_DOUBLE_EQ(1, T(1, 1));
    EXPECT_DOUBLE_EQ(0, T(1, 2));
    EXPECT_DOUBLE_EQ(0, T(1, 3));
    EXPECT_DOUBLE_EQ(0, T(2, 0));
    EXPECT_DOUBLE_EQ(0, T(2, 1));
    EXPECT_DOUBLE_EQ(1, T(2, 2));
    EXPECT_DOUBLE_EQ(0.5, T(2, 3));
    
    Eigen::VectorXd p = MyRobot.getToolPose(toolIndex, CR_EULER_MODE_XYZ);
    EXPECT_EQ(6, p.size());
    EXPECT_DOUBLE_EQ(3, p(0));
    EXPECT_DOUBLE_EQ(0, p(1));
    EXPECT_DOUBLE_EQ(0.5, p(2));
    EXPECT_DOUBLE_EQ(0, p(3));
    EXPECT_DOUBLE_EQ(0, p(4));
    EXPECT_DOUBLE_EQ(0, p(5));
    
    // get link frame
    Frame linkFrame;
    MyRobot.getLinkFrame(0, linkFrame);
    T = linkFrame.getTransformToParent();
    EXPECT_DOUBLE_EQ(1, T(0, 0));
    EXPECT_DOUBLE_EQ(0, T(0, 1));
    EXPECT_DOUBLE_EQ(0, T(0, 2));
    EXPECT_DOUBLE_EQ(0, T(0, 3));
    EXPECT_DOUBLE_EQ(0, T(1, 0));
    EXPECT_DOUBLE_EQ(1, T(1, 1));
    EXPECT_DOUBLE_EQ(0, T(1, 2));
    EXPECT_DOUBLE_EQ(0, T(1, 3));
    EXPECT_DOUBLE_EQ(0, T(2, 0));
    EXPECT_DOUBLE_EQ(0, T(2, 1));
    EXPECT_DOUBLE_EQ(1, T(2, 2));
    EXPECT_DOUBLE_EQ(0.5, T(2, 3));
}
 */



//
// OLD - DEPRECTATED MANIPULATOR CLASS
//
/*
// Setup the Manipulator robot and return the toolIndex
int setupRobot(world::Manipulator& MyRobot){
    FrameEuler* F0 = new FrameEuler();
    FrameEuler* F1 = new FrameEuler();
    FrameEuler* F2 = new FrameEuler();
    RigidBody* Link0 = new RigidBody();
    RigidBody* Link1 = new RigidBody();
    RigidBody* Link2 = new RigidBody();
    
    // Set info for Link 0 and add to MyRobot
    F0->setFreeVariable(CR_EULER_FREE_ANG_G);
    F0->setMode(CR_EULER_MODE_XYZ);
    F0->setPositionAndOrientation(0, 0, 0.5, 0, 0, 0);
    Link0->setFrame(F0);
    MyRobot.addLink(Link0);
    
    // Set info for Link 1 and add to MyRobot
    F1->setFreeVariable(CR_EULER_FREE_ANG_G);
    F1->setMode(CR_EULER_MODE_XYZ);
    F1->setPositionAndOrientation(1, 0, 0, 0, 0, 0);
    Link1->setFrame(F1);
    MyRobot.addLink(Link1);
    
    // Set info for Link 2 and add to MyRobot
    F2->setFreeVariable(CR_EULER_FREE_NONE);
    F2->setMode(CR_EULER_MODE_XYZ);
    F2->setPositionAndOrientation(2, 0, 0, 0, 0, 0);
    Link2->setFrame(F2);
    MyRobot.addLink(Link2);
    
    // create a tool frame and add to MyRobot
    FrameEuler* Tool = new FrameEuler();
    Tool->setMode(CR_EULER_MODE_XYZ);
    Tool->setPositionAndOrientation(0, 0, 0, 0, 0, 0);
    int toolIndex = MyRobot.addTool(2, Tool);
    return toolIndex;
}



//
// setup the manipulator
//
TEST(Manipulator, Setup){
    world::Manipulator MyRobot;
    FrameEuler* F0 = new FrameEuler();
    FrameEuler* F1 = new FrameEuler();
    FrameEuler* F2 = new FrameEuler();
    RigidBody* Link0 = new RigidBody();
    RigidBody* Link1 = new RigidBody();
    RigidBody* Link2 = new RigidBody();
    
    // Set info for Link 0 and add to MyRobot
    F0->setFreeVariable(CR_EULER_FREE_ANG_G);
    F0->setMode(CR_EULER_MODE_XYZ);
    F0->setPositionAndOrientation(0, 0, 0.5, 0, 0, 0);
    Link0->setFrame(F0);
    int i = MyRobot.addLink(Link0);
    EXPECT_EQ(0, i);
    
    // Set info for Link 1 and add to MyRobot
    F1->setFreeVariable(CR_EULER_FREE_ANG_G);
    F1->setMode(CR_EULER_MODE_XYZ);
    F1->setPositionAndOrientation(1, 0, 0, 0, 0, 0);
    Link1->setFrame(F1);
    i = MyRobot.addLink(Link1);
    EXPECT_EQ(1, i);
    
    // Set info for Link 2 and add to MyRobot
    F2->setFreeVariable(CR_EULER_FREE_NONE);
    F2->setMode(CR_EULER_MODE_XYZ);
    F2->setPositionAndOrientation(2, 0, 0, 0, 0, 0);
    Link2->setFrame(F2);
    i = MyRobot.addLink(Link2);
    EXPECT_EQ(2, i);
    
    // create a tool frame and add to MyRobot
    FrameEuler* Tool = new FrameEuler();
    Tool->setMode(CR_EULER_MODE_XYZ);
    Tool->setPositionAndOrientation(0, 0, 0, 0, 0, 0);
    int toolIndex = MyRobot.addTool(2, Tool);
    EXPECT_EQ(0, toolIndex);
}



//
// Set/GetConfiguration, GetNLinks, GetDOF
//
TEST(Manipulator, GetConfiguration){
    world::Manipulator MyRobot;
    int toolIndex = setupRobot(MyRobot);
    
    int n = MyRobot.getNumberOfLinks();
    int dof = MyRobot.getDegreesOfFreedom();
    Eigen::VectorXd q = MyRobot.getConfiguration();
    EXPECT_EQ(3, n);
    EXPECT_EQ(2, q.size());
    EXPECT_DOUBLE_EQ(2, dof);
    EXPECT_DOUBLE_EQ(0, q(0));
    EXPECT_DOUBLE_EQ(0, q(1));
    
    q << -M_PI / 4, M_PI / 4;
    MyRobot.setConfiguration(q);
    q = MyRobot.getConfiguration();
    EXPECT_DOUBLE_EQ(-M_PI / 4, q(0));
    EXPECT_DOUBLE_EQ(M_PI / 4, q(1));
}


//
// Get Forward kin
//
TEST(Manipulator, Kinematics){
    world::Manipulator MyRobot;
    int toolIndex = setupRobot(MyRobot);
    
    Eigen::MatrixXd fk;
    fk = MyRobot.getForwardKinematics();
    EXPECT_EQ(3, fk.rows());
    EXPECT_EQ(4, fk.cols());
    EXPECT_DOUBLE_EQ(0, fk(0, 0));
    EXPECT_DOUBLE_EQ(0, fk(1, 0));
    EXPECT_DOUBLE_EQ(0, fk(2, 0));
    EXPECT_DOUBLE_EQ(0, fk(0, 1));
    EXPECT_DOUBLE_EQ(0, fk(1, 1));
    EXPECT_DOUBLE_EQ(0.5, fk(2, 1));
    EXPECT_DOUBLE_EQ(1, fk(0, 2));
    EXPECT_DOUBLE_EQ(0, fk(1, 2));
    EXPECT_DOUBLE_EQ(0.5, fk(2, 2));
    EXPECT_DOUBLE_EQ(3, fk(0, 3));
    EXPECT_DOUBLE_EQ(0, fk(1, 3));
    EXPECT_DOUBLE_EQ(0.5, fk(2, 3));
}


//
// Get Jacobian
//
TEST(Manipulator, Jacobian){
    world::Manipulator MyRobot;
    int toolIndex = setupRobot(MyRobot);
    
    Eigen::MatrixXd J;
    J = MyRobot.jacobian(toolIndex, CR_EULER_MODE_XYZ);
    EXPECT_EQ(6, J.rows());
    EXPECT_EQ(2, J.cols());
    EXPECT_DOUBLE_EQ(0, J(0, 0));
    EXPECT_DOUBLE_EQ(3, J(1, 0));
    EXPECT_DOUBLE_EQ(0, J(2, 0));
    EXPECT_DOUBLE_EQ(0, J(3, 0));
    EXPECT_DOUBLE_EQ(0, J(4, 0));
    EXPECT_DOUBLE_EQ(1, J(5, 0));
    EXPECT_DOUBLE_EQ(0, J(0, 1));
    EXPECT_DOUBLE_EQ(2, J(1, 1));
    EXPECT_DOUBLE_EQ(0, J(2, 1));
    EXPECT_DOUBLE_EQ(0, J(3, 1));
    EXPECT_DOUBLE_EQ(0, J(4, 1));
    EXPECT_DOUBLE_EQ(1, J(5, 1));
    
    Eigen::Matrix<bool, 6, 1> pe;
    pe << true, true, false, false, false, true;
    J = MyRobot.jacobian(toolIndex, CR_EULER_MODE_XYZ, pe);
    EXPECT_EQ(3, J.rows());
    EXPECT_EQ(2, J.cols());
    EXPECT_DOUBLE_EQ(0, J(0, 0));
    EXPECT_DOUBLE_EQ(3, J(1, 0));
    EXPECT_DOUBLE_EQ(1, J(2, 0));
    EXPECT_DOUBLE_EQ(0, J(0, 1));
    EXPECT_DOUBLE_EQ(2, J(1, 1));
    EXPECT_DOUBLE_EQ(1, J(2, 1));
}


//
// Get Hessian
//
TEST(Manipulator, Hessian){
    world::Manipulator MyRobot;
    int toolIndex = setupRobot(MyRobot);
    
    // NOTE: the numerical hessian is noisy!
    Eigen::MatrixXd H;
    H = MyRobot.hessian(toolIndex, CR_EULER_MODE_XYZ);
    EXPECT_EQ(6, H.rows());
    EXPECT_EQ(2, H.cols());
    EXPECT_NEAR(-3, H(0, 0), 1e-3);
    EXPECT_DOUBLE_EQ(0, H(1, 0));
    EXPECT_DOUBLE_EQ(0, H(2, 0));
    EXPECT_DOUBLE_EQ(0, H(3, 0));
    EXPECT_DOUBLE_EQ(0, H(4, 0));
    EXPECT_DOUBLE_EQ(0, H(5, 0));
    EXPECT_NEAR(-2, H(0, 1), 1e-3);
    EXPECT_DOUBLE_EQ(0, H(1, 1));
    EXPECT_DOUBLE_EQ(0, H(2, 1));
    EXPECT_DOUBLE_EQ(0, H(3, 1));
    EXPECT_DOUBLE_EQ(0, H(4, 1));
    EXPECT_DOUBLE_EQ(0, H(5, 1));
    
    Eigen::Matrix<bool, 6, 1> pe;
    pe << true, true, false, false, false, true;
    H = MyRobot.hessian(toolIndex, CR_EULER_MODE_XYZ, pe);
    EXPECT_EQ(3, H.rows());
    EXPECT_EQ(2, H.cols());
    EXPECT_NEAR(-3, H(0, 0), 1e-3);
    EXPECT_DOUBLE_EQ(0, H(1, 0));
    EXPECT_DOUBLE_EQ(0, H(2, 0));
    EXPECT_NEAR(-2, H(0, 1), 1e-3);
    EXPECT_DOUBLE_EQ(0, H(1, 1));
    EXPECT_DOUBLE_EQ(0, H(2, 1));
}



//
// Get tool/link frames
//
TEST(Manipulator, GetFrames){
    world::Manipulator MyRobot;
    int toolIndex = setupRobot(MyRobot);
    
    // now get the transformation to the tool for the current configuration
    Eigen::Matrix4d T;
    Frame toolFrame;
    MyRobot.getToolFrame(toolIndex, toolFrame);
    T = toolFrame.getTransformToParent();
    EXPECT_DOUBLE_EQ(1, T(0, 0));
    EXPECT_DOUBLE_EQ(0, T(0, 1));
    EXPECT_DOUBLE_EQ(0, T(0, 2));
    EXPECT_DOUBLE_EQ(3, T(0, 3));
    EXPECT_DOUBLE_EQ(0, T(1, 0));
    EXPECT_DOUBLE_EQ(1, T(1, 1));
    EXPECT_DOUBLE_EQ(0, T(1, 2));
    EXPECT_DOUBLE_EQ(0, T(1, 3));
    EXPECT_DOUBLE_EQ(0, T(2, 0));
    EXPECT_DOUBLE_EQ(0, T(2, 1));
    EXPECT_DOUBLE_EQ(1, T(2, 2));
    EXPECT_DOUBLE_EQ(0.5, T(2, 3));
    
    Eigen::VectorXd p = MyRobot.getToolPose(toolIndex, CR_EULER_MODE_XYZ);
    EXPECT_EQ(6, p.size());
    EXPECT_DOUBLE_EQ(3, p(0));
    EXPECT_DOUBLE_EQ(0, p(1));
    EXPECT_DOUBLE_EQ(0.5, p(2));
    EXPECT_DOUBLE_EQ(0, p(3));
    EXPECT_DOUBLE_EQ(0, p(4));
    EXPECT_DOUBLE_EQ(0, p(5));
    
    // get link frame
    Frame linkFrame;
    MyRobot.getLinkFrame(0, linkFrame);
    T = linkFrame.getTransformToParent();
    EXPECT_DOUBLE_EQ(1, T(0, 0));
    EXPECT_DOUBLE_EQ(0, T(0, 1));
    EXPECT_DOUBLE_EQ(0, T(0, 2));
    EXPECT_DOUBLE_EQ(0, T(0, 3));
    EXPECT_DOUBLE_EQ(0, T(1, 0));
    EXPECT_DOUBLE_EQ(1, T(1, 1));
    EXPECT_DOUBLE_EQ(0, T(1, 2));
    EXPECT_DOUBLE_EQ(0, T(1, 3));
    EXPECT_DOUBLE_EQ(0, T(2, 0));
    EXPECT_DOUBLE_EQ(0, T(2, 1));
    EXPECT_DOUBLE_EQ(1, T(2, 2));
    EXPECT_DOUBLE_EQ(0.5, T(2, 3));
}
*/
