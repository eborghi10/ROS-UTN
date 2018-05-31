#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int16

MAX_MOTOR_VEL = 10.5 # m/s : motor maximum velocity (100 rpm)
SET_POINT_VEL = 0.75 * MAX_MOTOR_VEL 

def to_rpm(rad_s):
    return rad_s * 9.5492965964254

def to_rad_s(rpm):
    return rpm / 9.5492965964254

def to_motor(rad_s):
    return rad_s * 255. / MAX_MOTOR_VEL

def encoder_left_cb(data): # rad/s
    global encoder_left_msg
    encoder_left_msg = data.data

def encoder_right_cb(data):
    global encoder_right_msg
    encoder_right_msg = data.data

def on_shutdown_cb():
    rospy.loginfo("Shutting down robot")
    left_motor_pub.publish(0)
    right_motor_pub.publish(0)

def main():
    rospy.init_node('robot_controller', anonymous=True)

    rospy.Subscriber("encoder/left", Float32, encoder_left_cb)
    rospy.Subscriber("encoder/right", Float32, encoder_right_cb)
    global left_motor_pub
    global right_motor_pub
    left_motor_pub = rospy.Publisher('output/left', Int16, queue_size=10, latch=True)
    right_motor_pub = rospy.Publisher('output/right', Int16, queue_size=10, latch=True)

    error_left_pub = rospy.Publisher('error/left', Float32, queue_size=10, latch=True)
    error_right_pub = rospy.Publisher('error/right', Float32, queue_size=10, latch=True)

    rate = rospy.Rate(50) # Hz

    rospy.on_shutdown(on_shutdown_cb)

    encoder_left_msg = 0.
    encoder_right_msg = 0.

    while not rospy.is_shutdown():
        # https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_PID
        # Send velocities
        error_left = SET_POINT_VEL - encoder_left_msg
        error_left_pub.publish(error_left)
        left_motor_pub.publish(to_motor(error_left))
        
        error_right = SET_POINT_VEL - encoder_right_msg
        error_right_pub.publish(error_right)
        right_motor_pub.publish(to_motor(error_right))

        rate.sleep()

if __name__ == '__main__':
    main()
