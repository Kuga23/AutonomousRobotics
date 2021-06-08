#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist #message going to coppeliaSim/robot
from sensor_msgs.msg import Joy #message from the joystic
import numpy as np

def callback(data):
    twist = Twist()
    twist.linear.x = data.axes[1]
    twist.angular.z = -data.axes[0]
    pub.publish(twist)


def start():
    global pub
    pub = rospy.Publisher('/vrep/twistCommand', Twist, queue_size=1)
    rospy.Subscriber("joy", Joy, callback)
    rospy.init_node('Joy_to_robot')
    rospy.spin()

if __name__ == '__main__':
    start()
