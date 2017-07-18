from CoreRobotics import *
import numpy as np

print "**********************"
print "Running the CRTestFrameOffset"

MyRobot = CRManipulator()

F0 = CRFrameDh()
F1 = CRFrameDh()

Link0 = CRRigidBody()
Link1 = CRRigidBody()

F0.setFreeVariable(CR_DH_FREE_THETA)
F1.setFreeVariable(CR_DH_FREE_NONE)

F0.setMode(CR_DH_MODE_MODIFIED)
F1.setMode(CR_DH_MODE_MODIFIED)

F0.setParameters(0, 0, 0.2, 0, CR_PI / 2)
F1.setParameters(0.1, 0, 0, 0, 0);

Link0.setFrame(F0)
Link1.setFrame(F1)

MyRobot.addLink(Link0)
MyRobot.addLink(Link1)

dof = MyRobot.getDegreesOfFreedom()
jointAngles = MyRobot.getConfiguration()
print "MyRobot has", dof, "DOF, with joint angles =", jointAngles.T, "rad"

FwdKin = MyRobot.getForwardKinematics()
print "Forward Kinematics ="
print FwdKin
