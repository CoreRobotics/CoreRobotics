from CoreRobotics import *
import numpy as np

print "*************************************"
print "Demonstration of CRManipulator."

MyRobot = CRManipulator()

F0 = CRFrameEuler()
F1 = CRFrameEuler()
F2 = CRFrameEuler()
Link0 = CRRigidBody()
Link1 = CRRigidBody()
Link2 = CRRigidBody()

F0.setFreeVariable(CR_EULER_FREE_ANG_G)
F0.setMode(CR_EULER_MODE_XYZ)
F0.setPositionAndOrientation(0, 0, 0.5, 0, 0, 0)
Link0.setFrame(F0)
MyRobot.addLink(Link0)

F1.setFreeVariable(CR_EULER_FREE_ANG_G)
F1.setMode(CR_EULER_MODE_XYZ)
F1.setPositionAndOrientation(1, 0, 0, 0, 0, 0)
Link1.setFrame(F1)
MyRobot.addLink(Link1)

F2.setFreeVariable(CR_EULER_FREE_NONE)
F2.setMode(CR_EULER_MODE_XYZ)
F2.setPositionAndOrientation(2, 0, 0, 0, 0, 0)
Link2.setFrame(F2)
MyRobot.addLink(Link2)

Tool = CRFrameEuler()
Tool.setMode(CR_EULER_MODE_XYZ)
Tool.setPositionAndOrientation(0, 0, 0, 0, 0, 0)
toolIndex = MyRobot.addTool(2, Tool)

dof = MyRobot.getDegreesOfFreedom()
jointAngles = MyRobot.getConfiguration()
print "MyRobot has", dof, "DOF , with joint angles =", jointAngles.T, "rad"

FwdKin = MyRobot.getForwardKinematics()
Jacobian = MyRobot.jacobian(toolIndex, CR_EULER_MODE_XYZ)
print "Forward Kinematics ="
print FwdKin
print "Jacobian ="
print Jacobian

jointAngles = np.array([CR_PI / 4, -CR_PI / 2])
print "Set joint angles =", jointAngles.T, "rad"
MyRobot.setConfiguration(jointAngles)
FwdKin = MyRobot.getForwardKinematics()
Jacobian = MyRobot.jacobian(0, CR_EULER_MODE_XYZ)
print "Forward Kinematics ="
print FwdKin
print "Jacobian ="
print Jacobian

toolFrame = CRFrame()
MyRobot.getToolFrame(toolIndex, toolFrame)
T = toolFrame.getTransformToParent()
print "MyRobot tool has a transformation of"
print T

elems = np.array([1, 1, 0, 0, 0, 1], dtype = np.intc)
Jred = MyRobot.jacobian(toolIndex, CR_EULER_MODE_XYZ, elems)
print "MyRobot Jacobian (reduced) is"
print Jred

pose = MyRobot.getToolPose(toolIndex, CR_EULER_MODE_XYZ, elems)
print "MyRobot pose (reduced) is"
print pose
