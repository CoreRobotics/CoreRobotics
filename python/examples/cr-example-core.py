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
my_clock.sleep(0.001)
et = my_clock.getElapsedTime()

print("my_clock reports t (s): ", et)


# Many types derive from a base item class
my_item = CoreRobotics.Item()
my_item.setName(str("Data"))
print("my_item name is: ", my_item.getName())


# We can implement a control object from an abstract base step class
# These Step types are used extensively in other modules.
class MyStep(CoreRobotics.Step):
	def __init__(self):
		self.counter = 0
		CoreRobotics.Step.__init__(self)  # Expose the wrapped base class

	# A control loop calls the step method once every time step
	def step(self):
		print("step() call.")
		self.counter = self.counter + 1

	# This is called on start
	def onStart(self):
		self.counter = 0
		print("MyStep is starting.")

	# This is called on stop
	def onStop(self):
		print("MyStep stopped with step total: ", self.counter)
