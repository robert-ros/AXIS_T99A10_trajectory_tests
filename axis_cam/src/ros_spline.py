#!/usr/bin/env python

from scipy.interpolate import interp1d
import numpy as np
import time
import signal
import sys
import rospy
from axis_camera.msg import Axis


# Spline generator

x = np.linspace(0, 30, num=31, endpoint=True)
#y = 90*np.cos(-x**2/9.0)+90
y = 45*np.cos(2**np.pi*x*20)+45
f = interp1d(x, y)
f2 = interp1d(x, y, kind='cubic')
xnew = np.linspace(0, 30, num=131, endpoint=True)

count = int()
spline_time_now = float()
spline_time_last = float()
spline_elapsed_time = float()

count = 0
spline_time_now = 0
spline_time_last = 0
spline_elapsed_time = 0


import matplotlib.pyplot as plt
#plt.plot(x, y, 'o', xnew, f(xnew), '-', xnew, f2(xnew), 'o')
plt.plot(x, y, '-', xnew, f2(xnew), 'o')
plt.legend(['data','cubic'], loc='best')
plt.show(block=False)



def sigint_handler(sig, frame):
    print("Ending...")
    sys.exit()

#ROS Axis control 

rospy.init_node('axis_spline')

axis_state = Axis(pan=0)

spline_pub = rospy.Publisher('cmd', Axis, queue_size=10)

r = rospy.Rate(1)

# Set signal to exit using terminal
signal.signal(signal.SIGINT, sigint_handler)

while not rospy.is_shutdown():

    for t in xnew:
        
        count+=1
        print(count)

        spline_time_now = t 
        spline_elapsed_time = spline_time_now - spline_time_last
        spline_time_last = spline_time_now
        
        if spline_elapsed_time == 0:
            spline_elapsed_time = 1
        r = rospy.Rate(1/spline_elapsed_time)
        
        axis_state.pan = f2(t)
        spline_pub.publish(axis_state)
        print(axis_state.pan)

        r.sleep()


    print("END CODE")
    rospy.sleep(1)
 