from CoreRobotics import *
import numpy as np
import time

print "*************************************"
print "Demonstration of CRInverseKinematics."

convention = CR_EULER_MODE_XYZ

F0 = CRFrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G)
F1 = CRFrameEuler(1, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G)
F2 = CRFrameEuler(2, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G)
F3 = CRFrameEuler(1, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_NONE)

Link0 = CRRigidBody(F0)
Link1 = CRRigidBody(F1)
Link2 = CRRigidBody(F2)
Link3 = CRRigidBody(F3)

MyRobot = CRManipulator()

MyRobot.addLink(Link0)
MyRobot.addLink(Link1)
MyRobot.addLink(Link2)
attachLink = MyRobot.addLink(Link3)

Tool = CRFrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_NONE)
toolIndex = MyRobot.addTool(attachLink, Tool)

ikSolver = CRInverseKinematics(MyRobot, toolIndex, convention)

print "---------------------------------------------"
print "CASE 1: Use the default solver parameters."

q0 = np.array([0.1, -0.2, 0])
MyRobot.setConfiguration(q0)

p = np.array([2.5, 0, 0, 0, 0, 0])

qSolved = np.ndarray(q0.shape)

timer = time.clock()
result = ikSolver.solve(p, q0, qSolved)
et = time.clock() - timer

if result == CR_RESULT_SUCCESS:
	print "Non-singular solution found in", et, "s!"
	print qSolved

	MyRobot.setConfiguration(qSolved)
	fk = MyRobot.getForwardKinematics()

	print "The forward kinematics for this solution are:"
	print fk
else:
	print "The solution is singular."
	print qSolved

print "---------------------------------------------"
print "CASE 2: Change the default maximum iteration."

ikSolver.setMaxIter(100)

MyRobot.setConfiguration(q0)

timer = time.clock()
result = ikSolver.solve(p, q0, qSolved)
et = time.clock() - timer

if result == CR_RESULT_SUCCESS:
	print "Non-singular solution found in", et, "s!"
	print qSolved

	MyRobot.setConfiguration(qSolved)
	fk = MyRobot.getForwardKinematics()

	print "The forward kinematics for this solution are:"
	print fk
else:
	print "The solution is singular."
	print qSolved

print "---------------------------------------------"
print "CASE 3: Change the parameters."

ikSolver.setMaxIter(100)
ikSolver.setStepSize(0.2)
ikSolver.setTolerance(0.0001)

MyRobot.setConfiguration(q0)

timer = time.clock()
result = ikSolver.solve(p, q0, qSolved)
et = time.clock() - timer

if result == CR_RESULT_SUCCESS:
	print "Non-singular solution found in", et, "s!"
	print qSolved

	MyRobot.setConfiguration(qSolved)
	fk = MyRobot.getForwardKinematics()

	print "The forward kinematics for this solution are:"
	print fk
else:
	print "The solution is singular."
	print qSolved

print "---------------------------------------------"
print "CASE 4: Test a singular solution."

p = np.array([5, 0, 0, 0, 0, 0], dtype = np.float)

MyRobot.setConfiguration(q0)

timer = time.clock()
result = ikSolver.solve(p, q0, qSolved)
et = time.clock() - timer

if result == CR_RESULT_SUCCESS:
	print "Non-singular solution found in", et, "s!"
	print qSolved

	MyRobot.setConfiguration(qSolved)
	fk = MyRobot.getForwardKinematics()

	print "The forward kinematics for this solution are:"
	print fk
else:
	print "The solution is singular."
	print qSolved

print "---------------------------------------------"
print "CASE 5: Reduced pose vector."

elems = np.array([1, 1, 0, 0, 0, 1], dtype = np.intc)

pRed = np.array([2.5, 0, 0])

ikSolver.setStepSize(0.1)
ikSolver.setTolerance(0.001)

MyRobot.setConfiguration(q0)

timer = time.clock()
result = ikSolver.solve(pRed, elems, q0, qSolved)
et = time.clock() - timer

if result == CR_RESULT_SUCCESS:
	print "Non-singular solution found in", et, "s!"
	print qSolved

	MyRobot.setConfiguration(qSolved)
	fk = MyRobot.getForwardKinematics()

	print "The forward kinematics for this solution are:"
	print fk
else:
	print "The solution is singular."
	print qSolved

print "---------------------------------------------"
print "CASE 6: Single step convergance."

p = np.array([2.5, 0, 0, 0, 0, 0])

ikSolver.setMaxIter(1)

MyRobot.setConfiguration(q0)

q = q0

for i in range(100):
	timer = time.clock()
	result = ikSolver.solve(p, q, qSolved)
	et = time.clock() - timer

	q = qSolved

	if result == CR_RESULT_SUCCESS:
		print "Solution found in", et, "s!"

		MyRobot.setConfiguration(q)
		fk = MyRobot.getForwardKinematics()
	else:
		print "The solution is singular."
