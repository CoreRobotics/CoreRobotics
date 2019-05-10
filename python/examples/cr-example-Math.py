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
print("Wrap 3*pi [rad] = ", cr.Conversion.wrapToPi(3*np.pi))

# We have some integrators


# We have some matrix utilities
# cr.Matrix.reducedVector()

# We have some probaility methods
x = np.array([0, 0])[None,:]
mu = np.array([0, 0])[None,:]
Sigma = np.array([[1, 0], [0, 1]])
# print(cr.Probability.mvnpdf(x, mu, Sigma))