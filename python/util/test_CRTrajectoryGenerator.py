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
print("Demonstration of CRTrajectoryGenerator.")

# Define a trajectory generator
trajGen = CRTrajectoryGenerator()

# Initial and Final conditions
tf = 1.2
x0 = np.array([0., 0.])
v0 = np.array([0., -2.])
a0 = np.array([0., 0.])
xf = np.array([-0.4, 0.5])
vf = np.array([1.2, 1.0])
af = np.array([0., 0.])

# Compute the trajectory
trajGen.solve(x0, v0, a0, xf, vf, af, tf)

print("t (s) | Position | Velocity | Acceleration")
t = 0
while t <= tf:
	wp = trajGen.step(t)
	print("{:1.1f} | {:+1.3f}, {:+1.3f} | {:+1.3f}, {:+1.3f} | {:+1.3f}, {:+1.3f}".format(
		t,
		wp.position[0,0], wp.position[1,0],
		wp.velocity[0,0], wp.velocity[1,0],
		wp.acceleration[0,0], wp.acceleration[1,0]))
	t += 0.1
