#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int16

def encoder_left_cb(data): # rad/s
    global encoder_left_msg
    encoder_left_msg = data.data * 10

def encoder_right_cb(data):
    global encoder_right_msg
    encoder_right_msg = data.data * 10

def main():
    rospy.init_node('robot_controller', anonymous=True)

    rospy.Subscriber("encoder/left", Float32, encoder_left_cb)
    rospy.Subscriber("encoder/right", Float32, encoder_right_cb)
    left_motor_pub = rospy.Publisher('output/left', Int16, queue_size=1)
    right_motor_pub = rospy.Publisher('output/right', Int16, queue_size=1)

    rate = rospy.Rate(100) # Hz

    # Publish final Velocities
    left_motor_pub.publish(200)
    right_motor_pub.publish(200)

    global encoder_left_msg
    global encoder_right_msg
    encoder_left_msg = 0
    encoder_right_msg = 0

    while not rospy.is_shutdown():
        error_left = 200 - encoder_left_msg
        left_motor_pub.publish(200 + error_left)
        error_right = 200 - encoder_right_msg
        right_motor_pub.publish(200 + error_right)
        rate.sleep()

if __name__ == '__main__':
    main()
