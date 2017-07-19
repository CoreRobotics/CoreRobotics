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

# Import CoreRobotics and Numpy
from CoreRobotics import *
import numpy as np

print "**********************"
print "Running the CRTestFrameOffset"

# Create a new robot
MyRobot = CRManipulator()

# Create a couple Dh frames
F0 = CRFrameDh()
F1 = CRFrameDh()

# Create a couple of rigid body links
Link0 = CRRigidBody()
Link1 = CRRigidBody()

# Set the free variables
F0.setFreeVariable(CR_DH_FREE_THETA)
F1.setFreeVariable(CR_DH_FREE_NONE)

# Set the mode of the Dh frames
F0.setMode(CR_DH_MODE_MODIFIED)
F1.setMode(CR_DH_MODE_MODIFIED)

# Set the parameters of the Dh frames
F0.setParameters(0, 0, 0.2, 0, CR_PI / 2)
F1.setParameters(0.1, 0, 0, 0, 0);

# Add the frames to the links
Link0.setFrame(F0)
Link1.setFrame(F1)

# Add the links to MyRobot
MyRobot.addLink(Link0)
MyRobot.addLink(Link1)

# Now get the configuration values
dof = MyRobot.getDegreesOfFreedom()
jointAngles = MyRobot.getConfiguration()
print "MyRobot has", dof, "DOF, with joint angles =", jointAngles.T, "rad"

# Now get the Forward Kinematics
FwdKin = MyRobot.getForwardKinematics()
print "Forward Kinematics ="
print FwdKin
