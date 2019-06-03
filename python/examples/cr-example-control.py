# Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
# Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
# http://www.corerobotics.org

# Import the future print function for Python 2/3 compatability
from __future__ import print_function

# This script is an example of how to use CoreRobotics python bindings

import CoreRobotics
import numpy as np

# define starting and ending waypoints
dim = 2

wp0 = CoreRobotics.Waypoint(dim)
wp0.time = 0
wp0.position = np.array([0, 0])
wp0.velocity = np.array([-2, 0])
wp0.acceleration = np.array([0, 1])
wp0.jerk = np.array([0, 0])

wp1 = CoreRobotics.Waypoint(dim)
wp1.time = 5
wp1.position = np.array([1, 2])
wp1.velocity = np.array([-3, 0])
wp1.acceleration = np.array([2, 0])
wp1.jerk = np.array([0, 0])

# Set up tg parameters
tgparams = CoreRobotics.MinimumJerkParameters(2)
tgparams.solve(wp0, wp1)

print("The minimum jerk trajectory coefficients are")
print(tgparams.coefficients())

# create a min-jerk controller
# tg = CoreRobotics.TrajectoryGenerator(tgparams,  wp0)
