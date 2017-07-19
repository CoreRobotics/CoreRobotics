#=====================================================================
#
# Software License Agreement (BSD-3-Clause License)
# Copyright (c) 2017, CoreRobotics.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# 
# * Neither the name of CoreRobotics nor the names of its contributors
# may be used to endorse or promote products derived from this
# software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# 
# \project CoreRobotics Project
# \url     www.corerobotics.org
# \author  Cameron Devine
# \version 0.0
# 
#
#=====================================================================

# Import CoreRobotics Numpy and time
from CoreRobotics import *
import numpy as np
import time

print "*************************************"
print "Demonstration of CRInverseKinematics."

# Set the Euler convention we will use throughout the example
# Although CoreRobotics offers the flexibility to choose a
# different convention for each method, in general it is good
# to adopt the same convention throughout a problem for
# consistency.
convention = CR_EULER_MODE_XYZ

# ------------------------------------------
# Create the robot

# Create several rigid body links
F0 = CRFrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G)
F1 = CRFrameEuler(1, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G)
F2 = CRFrameEuler(2, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G)
F3 = CRFrameEuler(1, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_NONE)

Link0 = CRRigidBody(F0)
Link1 = CRRigidBody(F1)
Link2 = CRRigidBody(F2)
Link3 = CRRigidBody(F3)

# Create a new robot and add the links
MyRobot = CRManipulator()

MyRobot.addLink(Link0)
MyRobot.addLink(Link1)
MyRobot.addLink(Link2)
attachLink = MyRobot.addLink(Link3)

# Create a tool frame and add to MyRobot
Tool = CRFrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_NONE)
toolIndex = MyRobot.addTool(attachLink, Tool)

# ------------------------------------------
# Solve several invrese kinematics problems

# Set up an inverse kinematics object and attach the robot
ikSolver = CRInverseKinematics(MyRobot, toolIndex, convention)

# **********************
# CASE 1 : Solver should find a solution within default tolerance (1 mm),
# step size (1), and gain (0.1)
print "---------------------------------------------"
print "CASE 1: Use the default solver parameters."

# Set the initial configuration of the robot
q0 = np.array([0.1, -0.2, 0])
MyRobot.setConfiguration(q0)

# Define a set point pose
p = np.array([2.5, 0, 0, 0, 0, 0])

# Define the solution array
qSolved = np.ndarray(q0.shape)

# Now solve the inverse kinematics for the point
timer = time.time()
result = ikSolver.solve(p, q0, qSolved)
et = time.time() - timer

if result == CR_RESULT_SUCCESS:
	print "Non-singular solution found in", et, "s!"
	print qSolved

	# Now push the new joints through the robot to see if it worked
	MyRobot.setConfiguration(qSolved)
	fk = MyRobot.getForwardKinematics()

	print "The forward kinematics for this solution are:"
	print fk
else:
	print "The solution is singular."
	print qSolved

# **********************
# CASE 2:
print "---------------------------------------------"
print "CASE 2: Change the default maximum iteration."

# Change the maximum iterations
ikSolver.setMaxIter(100)

# I.C.
MyRobot.setConfiguration(q0)

# Now solve the inverse kinematics for the point
timer = time.time()
result = ikSolver.solve(p, q0, qSolved)
et = time.time() - timer

if result == CR_RESULT_SUCCESS:
	print "Non-singular solution found in", et, "s!"
	print qSolved

	# Now push the new joints through the robot to see if it worked
	MyRobot.setConfiguration(qSolved)
	fk = MyRobot.getForwardKinematics()

	print "The forward kinematics for this solution are:"
	print fk
else:
	print "The solution is singular."
	print qSolved

# **********************
# CASE 3: Change the tolerance and gain
print "---------------------------------------------"
print "CASE 3: Change the parameters."

# Change the solver parameters
ikSolver.setMaxIter(100)
ikSolver.setStepSize(0.2)
ikSolver.setTolerance(0.0001)

# I.C.
MyRobot.setConfiguration(q0)

# Now solve the inverse kinematics for the point
timer = time.time()
result = ikSolver.solve(p, q0, qSolved)
et = time.time() - timer

if result == CR_RESULT_SUCCESS:
	print "Non-singular solution found in", et, "s!"
	print qSolved

	# Now push the new joints through the robot to see if it worked
	MyRobot.setConfiguration(qSolved)
	fk = MyRobot.getForwardKinematics()

	print "The forward kinematics for this solution are:"
	print fk
else:
	print "The solution is singular."
	print qSolved

# **********************
# CASE 4: Assign a set point that is not reachable (singularity test)
print "---------------------------------------------"
print "CASE 4: Test a singular solution."

# Assign a set point outside the robot reach
p = np.array([5, 0, 0, 0, 0, 0], dtype = np.float)

# I.C.
MyRobot.setConfiguration(q0)

# Now solve the inverse kinematics for the point
timer = time.time()
result = ikSolver.solve(p, q0, qSolved)
et = time.time() - timer

if result == CR_RESULT_SUCCESS:
	print "Non-singular solution found in", et, "s!"
	print qSolved

	# Now push the new joints through the robot to see if it worked
	MyRobot.setConfiguration(qSolved)
	fk = MyRobot.getForwardKinematics()

	print "The forward kinematics for this solution are:"
	print fk
else:
	print "The solution is singular."
	print qSolved

# **********************
# CASE 5: Reduced pose vector
print "---------------------------------------------"
print "CASE 5: Reduced pose vector."

# Assign a reduced set point
elems = np.array([1, 1, 0, 0, 0, 1], dtype = np.intc)

pRed = np.array([2.5, 0, 0]) # (x, y, g)

# Change solver parameters
ikSolver.setStepSize(0.1)
ikSolver.setTolerance(0.001)

# I.C.
MyRobot.setConfiguration(q0)

# Now solve the inverse kinematics for the point
timer = time.time()
result = ikSolver.solve(pRed, elems, q0, qSolved)
et = time.time() - timer

if result == CR_RESULT_SUCCESS:
	print "Non-singular solution found in", et, "s!"
	print qSolved

	# Now push the new joint through the robot to see if it worked
	MyRobot.setConfiguration(qSolved)
	fk = MyRobot.getForwardKinematics()

	print "The forward kinematics for this solution are:"
	print fk
else:
	print "The solution is singular."
	print qSolved

# **********************
# CASE 6: Single step convergance
print "---------------------------------------------"
print "CASE 6: Single step convergance."

# Assign a set point
p = np.array([2.5, 0, 0, 0, 0, 0])

# Change solver parameters
ikSolver.setMaxIter(1)

# I.C.
MyRobot.setConfiguration(q0)

# Define a configuration
q = q0

# Now solve the inverse kinematics for the point
for i in range(100):
	timer = time.time()
	result = ikSolver.solve(p, q, qSolved)
	et = time.time() - timer

	q = qSolved

	if result == CR_RESULT_SUCCESS:
		print "Solution found in", et, "s!"

		# Now push the new joints through the robot to see if it worked
		MyRobot.setConfiguration(q)
		fk = MyRobot.getForwardKinematics()
	else:
		print "The solution is singular."

# -------------------------------------------------------------
