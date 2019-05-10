# Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
# Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
# http://www.corerobotics.org

# Import the future print function for Python 2/3 compatability
from __future__ import print_function

# This script is an example of how to use CoreRobotics python bindings

import CoreRobotics as cr
import numpy as np

# We can make a clock that sleeps and returns elapsed time
my_clock = cr.Clock()
my_clock.startTimer()
my_clock.sleep(0.1)
print("my_clock reports t (s): ", my_clock.getElapsedTime())

# Many types derive from a base item class
my_item = cr.Item()
my_item.setName(str("Items have names"))
print("my_item name is: ", my_item.getName())

# We can implement a control object from an abstract base step class
class MyStep(cr.Step):
	def __init__(self):
		self.counter = 0
	# A control loop calls the step method once every time step
	def step(self):
		self.counter = self.counter + 1
	# This is called on start
	def onStart(self):
		print("MyStep is starting.")
	# This is called on stop
	def onStop(self):
		print("MyStep stopped with step total: ", self.counter)

my_step = MyStep()
my_step.onStart()
my_step.step()
my_step.onStop()

# We can create loops to control the step items
dt = 0.01
my_loop = cr.Loop(dt)
# TODO(support for smart pointers):
# my_step2 = cr.Step()
# print(my_step2)
# print(my_loop)
# my_loop.attach(my_step2)


