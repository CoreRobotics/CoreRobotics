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
# 
#
#=====================================================================

# Import the future print function for Python 2/3 compatability
from __future__ import print_function

# Import CoreRobotics and Numpy
from CoreRobotics import *
import numpy as np

print("*************************************")
print("Demonstration of CRInverseKinematics.")

# Initialize Matricies
A = np.matrix([[1., 1.], [0., 1.]])
B = np.matrix([[0.5, 1.]]).T
C = np.matrix([[1., 0.]])
Q = np.matrix(0.1 * np.eye(2))
R = np.matrix([[0.3]])
x = np.matrix([[500., 0.]]).T
Sigma0 = np.matrix(0.01 * np.eye(2))
u = np.matrix([[-9.81]])

print("---------------------------------------------")
print("CASE 1: Discrete time Kalman filter.")

# Initialize the Kalman filter
kalman = CRKalmanFilter(A, B, C, Q, R, x, Sigma0)

for i in range(10):
	# Take a noisy measurement
	measurement = C * x + R * np.random.randn(1, 1)
	# Step the system forward
	x = A * x + B * u + Q * np.random.randn(2, 1)
	kalman.step(u, measurement)
	# Print the result
	print("At i =", i)
	print("The true state is", x.T)
	print("And the estimated state is", kalman.getState().T)
	print("With covariance")
	print(kalman.getCovariance())

print("---------------------------------------------")
print("CASE 2: Continuous time Kalman filter.")

# Test the system as continuous
x = np.matrix([[500., 0.]]).T
dt = 0.1

kalman = CRKalmanFilter(A, B, C, Q, R, x, Sigma0, dt)

for i in range(10):
	# Create a noisy measurement
	measurement = C * x + R * np.random.randn(1, 1)
	# Step the system forward
	xdot = A * x + B * u + Q * np.random.randn(2, 1)
	x = x + dt * xdot
	kalman.step(u, measurement)
	# Print the results
	print("At t =", i * dt)
	print("The true state is", x.T)
	print("And the estimated state is", kalman.getState().T)
	print("With covariance")
	print(kalman.getCovariance())
