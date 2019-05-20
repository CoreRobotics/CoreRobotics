# Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
# Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
# http://www.corerobotics.org

# Import the future print function for Python 2/3 compatability
from __future__ import print_function

# This script is an example of how to use CoreRobotics python bindings

import CoreRobotics as cr
import numpy as np

# We have some conversion utilities
print("Pi radians =", cr.Conversion.deg2rad(180))
print("Pi degrees =", cr.Conversion.rad2deg(np.pi))
print("Wrap 3*pi radians = ", cr.Conversion.wrapToPi(3*np.pi))
v1 = np.array([[0],[3*np.pi]])
print("Wrap [0 3]*pi radians = ", cr.Conversion.wrapToPi(v1))

# let's define a dynamical system
def linearSystem(t, x, u):
    return u - x

# we have different integrators available to the system
dt = 0.01
tf = 2.0
t = np.arange(tf / dt) * dt
x = np.zeros((2, t.size))
x[:,0] = np.array([[0, 1]])
u = np.array([[1],[-1]])
for k in range(t.size - 1):
    xk = np.reshape(x[:,k], (2, 1))
    x[:,k+1] = cr.Integration.rungeKuttaStep(
        linearSystem, t[k], xk, u, dt)

# We have some matrix utilities
# cr.Matrix.reducedVector()

# We have some probaility methods
x = np.array([[0], [0]])
mu = np.array([[0], [0]])
Sigma = np.array([[1, 0], [0, 1]])
# print(cr.Probability.mvnpdf(x, mu, Sigma))