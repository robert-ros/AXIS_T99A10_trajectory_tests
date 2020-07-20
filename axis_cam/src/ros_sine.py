#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
import signal
import sys
from axis_camera.msg import Axis
from std_msgs.msg import Float64

amplitude = 1.0 # axis cam uses relative speed 100%
frequency = 0.5 # Hz
max_time = 2.0 # seconds
samples = 100.0 #  trajecory points in graph (in max_time interval)


t = np.arange(0,max_time+max_time/samples , max_time/samples) 
y = amplitude * np.sin(2*np.pi*frequency*t)+0.5
print(t)
print(y)

plt.plot(t, y, 'o-')
plt.legend(['data'], loc='best')
plt.show(block = False)



def sigint_handler(sig, frame):
    print("Ending...")
    sys.exit()



#ROS Axis control 
rospy.init_node('axis_spline')
axis_state = Axis(pan=0)
command_state = Float64()
#spline_pub = rospy.Publisher('cmd', Axis, queue_size=10)
spline_pub = rospy.Publisher('pan_joint_position_controller/command', Float64, queue_size=10)
r = rospy.Rate( 1/(max_time/samples))

# Set signal to exit using terminal
signal.signal(signal.SIGINT, sigint_handler)

count = int()
count = 0

while not rospy.is_shutdown():

    for point in y:
        
        count+=1
        print(count)

        #axis_state.pan = point
        command_state.data = point
        #spline_pub.publish(axis_state)
        spline_pub.publish(command_state)
        #print(axis_state.pan)
        print(command_state.data)
        r.sleep()


    print("END TRAJECTORY")

 