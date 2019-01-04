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

# Import the future print function for Python 2/3 compatability
from __future__ import print_function

# Import CoreRobotics Numpy and time
from CoreRobotics import *
import numpy as np
import math
import copy

print("*************************************")
print("Demonstration of CRHardLimits.")

convention = CR_EULER_MODE_XYZ

# Define a robot
MyRobot = Manipulator()

# Define several frames
F0 = FrameEuler(0, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G)
F1 = FrameEuler(2, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G)
F2 = FrameEuler(-4, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G)
F3 = FrameEuler(1, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_ANG_G)

# Define the robot links
Link0 = RigidBody(F0)
Link1 = RigidBody(F1)
Link2 = RigidBody(F2)
Link3 = RigidBody(F3)

# Add the links to the robot
MyRobot.addLink(Link0)
MyRobot.addLink(Link1)
MyRobot.addLink(Link2)
linkIndex = MyRobot.addLink(Link3)

# Create a tool and add it
Tool = FrameEuler(1, 0, 0, 0, 0, 0, convention, CR_EULER_FREE_NONE)
toolIndex = MyRobot.addTool(linkIndex, Tool)

# Initialize a Hard Limits solver with the ik solver and the nullspace solver
solver = HardLimits(MyRobot, toolIndex, convention, True);

# Change IK solver maximum iterations
solver.getIKSolver().setMaxIter(20)

# Disable the nullspace solver for now
solver.useNullSpace(False)

# Set the pose elements
poseElements = np.array([1, 1, 0, 0, 0, 0], dtype = np.intc)
solver.setPoseElements(poseElements)

# Define the initial configuration
q0 = np.array([0, 0.1, 0.2, 0.3])

# Define and set the goal location
goal = np.array([0, 4.5])

print("Setting goal tool pose to ", goal, " (x, y)")

solver.setToolPose(goal)

# Run the solver
q = copy.copy(q0)
for i in range(5):
	solver.setQ0(q)
	solver.solve(q)

# Display the results
print("Calculated joint configuration without joint limits ", q)

MyRobot.setConfiguration(q)
print("Resulting tool pose ", MyRobot.getToolPose(toolIndex, convention, poseElements).T[0])

# Set joint limits
solver.setJointLimits(2, -1, 1)

# Run the solver
q = copy.copy(q0)
for i in range(5):
	solver.setQ0(q)
	solver.solve(q)

# Display the results
print("Calculated joint configuration woth joint limits ", q)

MyRobot.setConfiguration(q)
print("Resulting tool pose ", MyRobot.getToolPose(toolIndex, convention, poseElements).T[0])

# Enable the nullspace solver
solver.useNullSpace(True)

# Set the desired nullspace joint motion
desiredJointMotion = np.array([-1.0, 0, 0, 0])

print("Setting nullspace desired joint motion to ", desiredJointMotion)

solver.setJointMotion(desiredJointMotion)

# Run the solver
q = copy.copy(q0)
for i in range(5):
	solver.setQ0(q)
	solver.solve(q)

# Display the results
print("Calculated joint configuration woth joint limits and nullspace control ", q)

MyRobot.setConfiguration(q)
print("Resulting tool pose ", MyRobot.getToolPose(toolIndex, convention, poseElements).T[0])

# Set the initial condition to a point outside the limits
q =np.array([0, 0.1, math.pi, 0.3])
print("Setting initial condition outisde the limits, ", q)
solver.setQ0(q)

# Run the solver
result = solver.solve(q)

# Display the results
print("Recieved the result", end = " ")
if result == CR_RESULT_SUCCESS:
	print("CR_RESULT_SUCCESS (success)", end = " ")
elif result == CR_RESULT_SINGULAR:
	print("CR_RESULT_SINGULAR (singular jacobian)", end = " ")
elif result == CR_RESULT_BAD_IC:
	print("CR_RESULT_BAD_IC (bad initial conditions)", end = " ")
print("with joint configuration ", q)
