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

# This script is an example of how to use the built in python libraries
# to emulate the functionality of CRCore

# Use the built in time and threading libraries
import time
from threading import Thread
from CoreRobotics import *
import numpy as np

memoryName = "MyMemory"

print("**********************")
print("Demonstration of CRCore")

MyClock = time.time()
time.sleep(0.1)
t = time.time() - MyClock

print("t =", t)

# Open a shared memory object
mem = CRSharedMemory(memoryName, CR_MANAGER_SERVER)

# Create a vector of data
v = np.array([[0.0, 0.8]]).T

# Add a signal
mem.addSignal("signal_1", v)

# Callback for the first thread
def callback1():
	# Open some shared memory as client
	mem = CRSharedMemory(memoryName, CR_MANAGER_CLIENT)

	dt = 0.1
	for i in range(10):
		c = time.time()
		v = mem.get("signal_1")
		print("Thread 1: i = {}, signal = {}, {}".format(i + 1, v[0,0], v[1,0]))
		t = time.time() - c
		time.sleep(max(dt - t, 0))

# Callback for the second thread
def callback2():
	# Open some shared memory as client
	mem = CRSharedMemory(memoryName, CR_MANAGER_CLIENT)
	# Create a vector of data
	v = np.array([[0.1, 0.4]]).T

	dt = 0.25
	for i in range(4):
		c = time.time()
		v = i * v + v
		mem.set("signal_1", v)
		print("Thread 2: i = {}, signal = {}, {}".format(i + 1, v[0,0], v[1,0]))
		t = time.time() - c
		time.sleep(max(dt - t, 0))

# Create the threads
myThread1 = Thread(target = callback1)
myThread2 = Thread(target = callback2)

# Start the threads
myThread1.start()
myThread2.start()

# Wait for the threads to complete
while myThread1.isAlive() and myThread2.isAlive():
	pass

# Remove the signal
mem.removeSignal("Signal_1")
