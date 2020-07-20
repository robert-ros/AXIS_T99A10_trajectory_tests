#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from axis_camera.msg import Axis


class Viewer:

    def __init__(self):

        rospy.init_node('cmd_viewer_node', anonymous=True)
        rospy.Subscriber("cmd", Axis, self.cmd_callback, queue_size=1)


    def cmd_callback(self, msg):

        x_angle = msg.pan
        y_angle = msg.tilt
        print(x_angle)

        rospy.loginfo( "Pan: %f   Tilt: %f", x_angle, y_angle)


def main():

    axis_viewer = Viewer()

    while not rospy.is_shutdown():

        dummy = 0


if __name__ == '__main__':
    try:
        print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        main()
    except rospy.ROSInterruptException:
        pass