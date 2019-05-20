# Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
# Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
# http://www.corerobotics.org

# Import the future print function for Python 2/3 compatability
from __future__ import print_function

# This script is an example of how to use CoreRobotics python bindings

import CoreRobotics
import numpy as np

# We can make a clock that sleeps and returns elapsed time
my_clock = CoreRobotics.Clock()
my_clock.startTimer()
my_clock.sleep(0.1)
print("my_clock reports t (s): ", my_clock.getElapsedTime())

# Many types derive from a base item class
my_item = CoreRobotics.Item()
my_item.setName(str("Amelia Pond"))
print("my_item name is: ", my_item.getName())

# We can implement a control object from an abstract base step class
class MyStep(CoreRobotics.Step):
	def __init__(self):
		self.counter = 0
		CoreRobotics.Step.__init__(self)  # Needed to expose the wrapped base class
	# A control loop calls the step method once every time step
	def step(self):
		self.counter = self.counter + 1
	# This is called on start
	def onStart(self):
		print("MyStep is starting.")
	# This is called on stop
	def onStop(self):
		print("MyStep stopped with step total: ", self.counter)

# We can create loops to control the step items
dt = 0.1
my_loop = CoreRobotics.Loop(dt)

# Lets use the loop to run the step
my_step = MyStep()
my_loop.attach(my_step)
my_loop.start()
my_clock.sleep(0.5)
my_loop.stop()