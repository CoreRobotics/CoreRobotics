# Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
# Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
# http://www.corerobotics.org

# Import the future print function for Python 2/3 compatability
from __future__ import print_function

# This script is an example of how to use CoreRobotics python bindings

import CoreRobotics
import numpy as np

# create a clock
clock = CoreRobotics.Clock()

# We can create loops to control the step items
dt = 0.1
my_loop = CoreRobotics.Loop(dt)

# Lets use the loop to run the step
# TODO: https://gitlab.com/powan/CoreRobotics/issues/52
# my_step = MyStep()   # This has non-deterministic problems - hanging, 
# not stepping, etc...
my_step  = CoreRobotics.Step()
my_loop.attach(my_step)
my_loop.setPriority(CoreRobotics.CR_PRIORITY_HIGH)

my_loop.start()
clock.sleep(1.0)
my_loop.stop()
