#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, UInt16

ping_ = 100

def callback(data):
    global ping_
    ping_ = data.data

def listener():

    rospy.init_node('ping_listener', anonymous=True)
    rospy.Subscriber("ping", Float32, callback)
    left_motor_pub = rospy.Publisher('output/left', UInt16, queue_size=1)
    right_motor_pub = rospy.Publisher('output/right', UInt16, queue_size=1)
    rate = rospy.Rate(4)
    while not rospy.is_shutdown():
        if ping_ > 0.5:
            right_motor_pub.publish(100)
            left_motor_pub.publish(100)
        else:
            right_motor_pub.publish(0)
            left_motor_pub.publish(0)
        rate.sleep()

if __name__ == '__main__':
    listener()
