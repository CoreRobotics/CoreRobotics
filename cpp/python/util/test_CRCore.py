import time
from threading import Thread

print "**********************"
print "Running the test_CRCore";

def callback1():
	dt = 0.1
	for i in range(10):
		c = time.clock()
		print "Thread 1: i =", i + 1
		t = time.clock() - c
		time.sleep(dt - t)

def callback2():
	dt = 0.25
	for i in range(4):
		c = time.clock()
		print "Thread 2: i =", i + 1
		t = time.clock() - c
		time.sleep(dt - t)

myThread1 = Thread(target = callback1)
myThread2 = Thread(target = callback2)

myThread1.start()
myThread2.start()

time.sleep(1)
