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

# This script is an example of how to use the built in python libraries
# to emulate the functionality of CRCore

# Use the built in time and threading libraries
import time
from threading import Thread

print "**********************"
print "Running the test_CRCore";

MyClock = time.time()
time.sleep(0.1)
t = time.time() - MyClock

print "t =", t

# Callback for the first thread
def callback1():
	dt = 0.1
	for i in range(10):
		c = time.time()
		print "Thread 1: i = {}".format(i + 1)
		t = time.time() - c
		time.sleep(dt - t)

# Callback for the second thread
def callback2():
	dt = 0.25
	for i in range(4):
		c = time.time()
		print "Thread 2: i = {}".format(i + 1)
		t = time.time() - c
		time.sleep(dt - t)

# Create the threads
myThread1 = Thread(target = callback1)
myThread2 = Thread(target = callback2)

# Start the threads
myThread1.start()
myThread2.start()

# Wait for the threads to complete
time.sleep(1)
