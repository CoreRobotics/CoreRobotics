#include <iostream>
#include "CoreRobotics.hpp"

using namespace CoreRobotics;

void test_CRNullSpace(void) {

	std::cout << "*************************************\n";
	std::cout << "Demonstration of CRNullSpace.\n";

	CRManipulator* MyRobot = new CRManipulator();

	CRFrameEuler* F0 = new CRFrameEuler();
	CRFrameEuler* F1 = new CRFrameEuler();
	CRFrameEuler* F2 = new CRFrameEuler();
	CRRigidBody* Link0 = new CRRigidBody();
	CRRigidBody* Link1 = new CRRigidBody();
	CRRigidBody* Link2 = new CRRigidBody();

	// Set info for Link 0 and add to MyRobot
	F0->setFreeVariable(CR_EULER_FREE_ANG_G);
	F0->setMode(CR_EULER_MODE_XYZ);
	F0->setPositionAndOrientation(0, 0, 0.5, 0, 0, 0);
	Link0->setFrame(F0);
	MyRobot->addLink(Link0);

	// Set info for Link 1 and add to MyRobot
	F1->setFreeVariable(CR_EULER_FREE_ANG_G);
	F1->setMode(CR_EULER_MODE_XYZ);
	F1->setPositionAndOrientation(1, 0, 0, 0, 0, 0);
	Link1->setFrame(F1);
	MyRobot->addLink(Link1);

	// Set info for Link 2 and add to MyRobot
	F2->setFreeVariable(CR_EULER_FREE_NONE);
	F2->setMode(CR_EULER_MODE_XYZ);
	F2->setPositionAndOrientation(2, 0, 0, 0, 0, 0);
	Link2->setFrame(F2);
	MyRobot->addLink(Link2);


	// create a tool frame and add to MyRobot
	CRFrameEuler* Tool = new CRFrameEuler();
	Tool->setMode(CR_EULER_MODE_XYZ);
	Tool->setPositionAndOrientation(0, 0, 0, 0, 0, 0);
	int toolIndex = MyRobot->addTool(2, Tool);

	CRNullSpace nullSpaceSolver = CRNullSpace(MyRobot, 0, CR_EULER_MODE_XYZ);

	Eigen::VectorXd toolPose;
	toolPose = MyRobot->getToolPose(0, CR_EULER_MODE_XYZ);
	std::cout << "Tool Pose\n" << toolPose << std::endl;

	Eigen::VectorXd jointVel(2);
	jointVel << 1, -0.5;
	Eigen::VectorXd qNull = nullSpaceSolver.solve(jointVel, MyRobot->getConfiguration());
	std::cout << "NullSpace move\n" << qNull << std::endl;

	MyRobot->setConfiguration(qNull);
	toolPose = MyRobot->getToolPose(0, CR_EULER_MODE_XYZ);
	std::cout << "Tool Pose\n" << toolPose << std::endl;

	Eigen::Matrix<bool, 6, 1> poseElements;
	poseElements << 0, 1, 0, 0, 0, 0;

	qNull = nullSpaceSolver.solve(jointVel, poseElements, MyRobot->getConfiguration());
	std::cout << "NullSpace move\n" << qNull << std::endl;

	MyRobot->setConfiguration(qNull);
	toolPose = MyRobot->getToolPose(0, CR_EULER_MODE_XYZ);
	std::cout << "Tool Pose\n" << toolPose << std::endl;
}
