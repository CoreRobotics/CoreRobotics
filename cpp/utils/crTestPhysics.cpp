//
//  crTestPhysics.cpp
//  testStuite
//
//  Created by Parker Owan on 1/30/17.
//  Copyright © 2017 CoreRobotics. All rights reserved.
//

#include <iostream>
#include "CoreRobotics.hpp"


// Use the CoreRobotics namespace
using namespace CoreRobotics;
using namespace Eigen;
using namespace std;

const double CR_PI = 3.1415926535897932384626433832795;

#include <unistd.h>


int crTestPhysics(int N){
    
    /*
    cout << "The average compute times (us) for the Frame() classes\n";
    
    
    //-----------------------------------------------------------------
    // Init the test variables
    Matrix3d Rset, Rget;
    Vector3d Tset, Tget;
    Matrix4d T1, T2;
    Vector3d P1;
    Matrix3Xd P2;
    
    //-----------------------------------------------------------------
    // Set soime values
    Rset << cos(0.7), -sin(0.7), 0, sin(0.7), cos(0.7), 0, 0, 0, 1;
    Tset << 4, 5, 7;
    P1 << 9, 5, 6;
    
    
    //-----------------------------------------------------------------
    // Setup the timer
    CRClock myClock;
    double t;
    */
    
    //-----------------------------------------------------------------
    // Test the CRFrame Class
    /*
    CoreRobotics::CRFrame Frame;
    
    Eigen::Matrix3d Rot;
    Eigen::Vector3d Trans;
    Rot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    Trans << -2, 2, 3;
    
    Frame.setRotationAndTranslation(Rot, Trans);
    
    Eigen::Matrix4d T;
    Frame.getTransformToParent(T);
    std::cout << "Transformation to parent\n" << T << std::endl;
    
    Eigen::Vector3d p, y;
    p << 5, 6, 7;
    Frame.transformToParent(p, y);
    std::cout << "Point " << p.transpose() << " transformed to "
        << y.transpose() << std::endl;
    */
    
    /*
    CoreRobotics::CRFrameDh Frame;
    Frame.setParameters(0.1, 0.0, 2.1, 0.7);
    Eigen::Matrix4d T;
    Frame.getTransformToParent(T);
    std::cout << "Transformation to parent\n" << T << std::endl;
    Eigen::Vector3d p, y;
    p << 5, 6, 7;
    Frame.transformToParent(p, y);
    std::cout << "Point " << p.transpose() << " transformed to "
    << y.transpose() << std::endl;
    */
    
    
    
    
    
    /*
    
    myClock.startTimer();
    //for (int i = 0; i < N; i++) {
    //    Frame1.transformToChild(P1, P2);
    //    Frame1.transformToParent(P2, P2);
    //}
    myClock.getElapsedTime(t);
    cout << t/double(N)/2.0*1.0e6 << endl;
    
    //-----------------------------------------------------------------
    // Test the CRFrameEuler Class
    CRFrameEuler Frame2;
    Frame2.freeVar = CR_EULER_FREE_ANG_A;
    Frame2.setMode(CR_EULER_MODE_XYZ);
    Frame2.setPositionAndOrientation(1, 2, 3, 0, 0, 0.7);
    Frame2.getRotationAndTranslation(Rget, Tget);
    Frame2.getTransformToChild(T1);
    Frame2.getTransformToParent(T2);
    myClock.startTimer();
    //for (int i = 0; i < N; i++) {
    //    Frame2.transformToChild(P1, P2);
    //    Frame2.transformToParent(P2, P2);
    //}
    myClock.getElapsedTime(t);
    cout << t/double(N)/2.0*1.0e6 << endl;
    
    //-----------------------------------------------------------------
    // Test the CRFrameDh Class
    CRFrameDh Frame3;
    Frame3.setMode(CR_DH_MODE_CLASSIC);
    Frame3.setParameters(5, 0.7, 2, 1.2);
    Frame3.getRotationAndTranslation(Rget, Tget);
    Frame3.getTransformToChild(T1);
    Frame3.getTransformToParent(T2);
    myClock.startTimer();
    //for (int i = 0; i < N; i++) {
    //    Frame3.transformToChild(P1, P2);
    //    Frame3.transformToParent(P2, P2);
    //}
    myClock.getElapsedTime(t);
    cout << t/double(N)/2.0*1.0e6 << endl;
    
    //-----------------------------------------------------------------
    // Test the RigidBody class
    CRRigidBody Link;
    Link.frame = &Frame2;
    Link.frame->getTransformToChild(T1);
    
    
    
    CRFrameDh* F1 = new CRFrameDh();
    F1->freeVar = CR_DH_FREE_NONE;
    cout << F1->isDriven() << endl;
    
    F1->freeVar = CR_DH_FREE_THETA;
    cout << F1->isDriven() << endl;
    */
    
    
    //-----------------------------------------------------------------
    // Now test the manipulator class
    /*
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
    
    */
    
    /*
    Eigen::MatrixXd fk;
    Robot.getForwardKinematics(fk);
    cout << "Robot forward kinematics = " << endl;
    cout << fk << endl;
    
    // Now loop a bunch of times and see how the execution takes
    int runs = 1;
    MatrixXd J;
    Matrix<double,1,1> time;
    
    for (int k = 0; k < runs; k++) {
        myClock.startTimer();
        Robot.setConfiguration(jointAngles);
        Robot.getForwardKinematics(fk);
        Robot.getJacobian(J);
        myClock.getElapsedTime(time(k));
    }
    cout << "Mean loop compute time = " << time.sum()/double(runs) << " s" << endl;
    cout << "Max  loop compute time = " << time.maxCoeff() << " s" << endl;
    cout << "Min  loop compute time = " << time.minCoeff() << " s" << endl;
    
    cout << "Robot Jacobian = " << endl;
    cout << J << endl;
     */
    
    CRClock MyClock;
    double t;
    useconds_t wait = 1.0e6;
    MyClock.startTimer();
    usleep(wait);
    MyClock.getElapsedTime(t);
    std::cout << "Elapsed time = " << t << " sec" << std::endl;
    usleep(wait);
    MyClock.getElapsedTime(t);
    std::cout << "Elapsed time = " << t << " sec" << std::endl;
    
    
    return 0;
}
