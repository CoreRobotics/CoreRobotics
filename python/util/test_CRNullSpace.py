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

print("*************************************")
print("Demonstration of CRNullSpace.")

MyRobot = CRManipulator()

# Initialize translation frames
F0 = CRFrameEuler()
F1 = CRFrameEuler()
F2 = CRFrameEuler()
F3 = CRFrameEuler()
F4 = CRFrameEuler()
F5 = CRFrameEuler()
F6 = CRFrameEuler()

# Initialize links
Link0 = CRRigidBody()
Link1 = CRRigidBody()
Link2 = CRRigidBody()
Link3 = CRRigidBody()
Link4 = CRRigidBody()
Link5 = CRRigidBody()
Link6 = CRRigidBody()

# Set translation frames and DH Modified
F0.setMode(CR_EULER_MODE_XYZ)
F1.setMode(CR_EULER_MODE_XYZ)
F2.setMode(CR_EULER_MODE_XYZ)
F3.setMode(CR_EULER_MODE_XYZ)
F4.setMode(CR_EULER_MODE_XYZ)
F5.setMode(CR_EULER_MODE_XYZ)
F6.setMode(CR_EULER_MODE_XYZ)

# Set tralation frame free variables
F0.setFreeVariable(CR_EULER_FREE_ANG_G)
F1.setFreeVariable(CR_EULER_FREE_ANG_G)
F2.setFreeVariable(CR_EULER_FREE_ANG_A)
F3.setFreeVariable(CR_EULER_FREE_ANG_G)
F4.setFreeVariable(CR_EULER_FREE_ANG_B)
F5.setFreeVariable(CR_EULER_FREE_ANG_A)
F6.setFreeVariable(CR_EULER_FREE_NONE)

# lengths
d1	= 129.275
d2	= 62.68
d3	= 70.0
d4	= 4.135
d5	= 201.07
d6	= 10.57
d7	= 163.24
d8	= 31.25
d9	= 75.47
d10	= 39.36

# Set Euler Angles and Position Offsets
#							  dx		dy		dz		tx				ty		tz
F0.setPositionAndOrientation(0.0,		0.0,	d1,		0.0,			0.0,	0.0)
F1.setPositionAndOrientation(0.0,		0.0,	0.0,	CR_PI/2.0,		0.0,	0.0)
F2.setPositionAndOrientation(d3,		d4,		d2,		0.0,			0.0,	0.0)
F3.setPositionAndOrientation(d5,		0.0,	-d6,	0.0,			0.0,	0.0)
F4.setPositionAndOrientation(d7,		0.0,	-d8,	-CR_PI/2.0,		0.0,	0.0)
F5.setPositionAndOrientation(d9,		d10,	0.0,	0.0,			0.0,	0.0)
F6.setPositionAndOrientation(0.0,		0.0,	0.0,	0.0,			0.0,	0.0)

# Add frames to links
Link0.setFrame(F0)
Link1.setFrame(F1)
Link2.setFrame(F2)
Link3.setFrame(F3)
Link4.setFrame(F4)
Link5.setFrame(F5)
Link6.setFrame(F6)

# Add links to robot
linkIndex0 = MyRobot.addLink(Link0);
linkIndex1 = MyRobot.addLink(Link1);
linkIndex2 = MyRobot.addLink(Link2);
linkIndex3 = MyRobot.addLink(Link3);
linkIndex4 = MyRobot.addLink(Link4);
linkIndex5 = MyRobot.addLink(Link5);
linkIndex6 = MyRobot.addLink(Link6);

# Create a tool frame and add to MyRobot
Tool = CRFrameEuler()
Tool.setFreeVariable(CR_EULER_FREE_NONE)
Tool.setMode(CR_EULER_MODE_XYZ)
Tool.setPositionAndOrientation(90.21, 0.0, -107.43, -CR_PI / 8.0, 0.0, 0.0)
toolIndex = MyRobot.addTool(linkIndex6, Tool)

# Initialize the solver
nullSpaceSolver = CRNullSpace(MyRobot, toolIndex, CR_EULER_MODE_XYZ)

# Set the robot orientation
InitJoints = np.array([0.0, 2.0, 0.0, -2, 0.0, CR_PI / 8.0])
MyRobot.setConfiguration(InitJoints);

# Compute the initial tool pose
toolPose = MyRobot.getToolPose(toolIndex, CR_EULER_MODE_XYZ)
print("Initial Tool Pose\n", toolPose.T)

# Find a nullspace movement
jointMotion = np.array([1., 0, 0, 0, 0, 0])
nullJointMotion = np.ndarray(jointMotion.shape)
result = nullSpaceSolver.solve(jointMotion, InitJoints, nullJointMotion)
if result != CR_RESULT_SUCCESS:
	print("Singular Jacobian")
print("NullSpace joint movements\n", nullJointMotion.T)

# Compute the final tool pose
MyRobot.setConfiguration(InitJoints + nullJointMotion);
toolPose = MyRobot.getToolPose(toolIndex, CR_EULER_MODE_XYZ);
print("Final Tool Pose\n", toolPose.T)

# Set pose elements
poseElements = np.array([1, 1, 1, 1, 1, 0], dtype = np.intc)

# Find a nullspace movement with pose elements
result = nullSpaceSolver.solve(jointMotion, InitJoints, poseElements, nullJointMotion)
if result != CR_RESULT_SUCCESS:
	print("Singular Jacobian")
print("Reduced NullSpace joint movements\n", nullJointMotion.T)

# Compute the final tool pose
MyRobot.setConfiguration(InitJoints + nullJointMotion)
toolPose = MyRobot.getToolPose(toolIndex, CR_EULER_MODE_XYZ)
print("Final Tool Pose\n", toolPose.T)
