#!/usr/bin/env python

import rospy
import threading
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Quaternion, Pose
from nav_msgs.msg import Odometry

WHEEL_RADIUS = 3.1 / 1E2 # 3.1 cm
WHEEL_SEPARATION = 14.6 / 1E2 # 14.6 cm

LEFT = 0
RIGHT = 1

old_angle = [0, 0]
dEncoder = [0, 0]
accum = [0, 0]

x = 0
y = 0
theta = 0

def odometryClassical():
    # Linear and angular displacements
    lock.acquire()
    dS = WHEEL_RADIUS * (dEncoder[LEFT] + dEncoder[RIGHT]) / 2.
    dTheta = WHEEL_RADIUS * (dEncoder[RIGHT] - dEncoder[LEFT]) / WHEEL_SEPARATION
    lock.release()
    global x, y, theta
    x += dS * np.cos(theta)
    y += dS * np.sin(theta)
    theta += dTheta
    return x, y, theta

def odometryICC():
    lock.acquire()
    diff = dEncoder[RIGHT] - dEncoder[LEFT]
    if abs(diff) < 1E-4:
        R = 0
    else:
        R = (WHEEL_SEPARATION / 2) * (dEncoder[LEFT] - dEncoder[RIGHT]) / diff
    omegaDT = WHEEL_RADIUS * diff / WHEEL_SEPARATION
    lock.release()
    global x, y, theta
    ICC = [x - R * np.sin(theta), y + R * np.cos(theta)]
    x = np.cos(omegaDT) * (x-ICC[0]) - np.sin(omegaDT) * (y-ICC[1]) + ICC[0]
    y = np.sin(omegaDT) * (x-ICC[0]) + np.cos(omegaDT) * (y-ICC[1]) + ICC[1]
    theta += omegaDT
    return x, y, theta

def unwrap(angle_t1, wheel):
    global old_angle, accum
    diff = angle_t1 - old_angle[LEFT if wheel is "left" else RIGHT]
    if diff > np.pi:
        accum[LEFT if wheel is "left" else RIGHT] -= 2*np.pi
    elif diff < -np.pi:
        accum[LEFT if wheel is "left" else RIGHT] += 2*np.pi
    old_angle[LEFT if wheel is "left" else RIGHT] = angle_t1

def encoder_left_cb(msg):
    # Store old unwrapped angle (for velocity calc)
    old_angle_unwrapped = accum[LEFT] + old_angle[LEFT]
    # Unwrap new angle
    unwrap(msg.data, "left")
    new_angle_unwrapped = accum[LEFT] + msg.data
    pos_left_pub.publish(Float32(new_angle_unwrapped))
    # Calculate displacement
    lock.acquire()
    dEncoder[LEFT] = new_angle_unwrapped - old_angle_unwrapped
    rel_pos_left_pub.publish(Float32(dEncoder[LEFT]))
    lock.release()

def encoder_right_cb(msg):
    old_angle_unwrapped = accum[RIGHT] + old_angle[RIGHT]
    unwrap(msg.data, "right")
    new_angle_unwrapped = accum[RIGHT] + msg.data
    pos_right_pub.publish(Float32(new_angle_unwrapped))
    # Calculate displacement
    lock.acquire()
    dEncoder[RIGHT] = new_angle_unwrapped - old_angle_unwrapped
    rel_pos_right_pub.publish(Float32(dEncoder[RIGHT]))
    lock.release()

def init():
    global lock
    lock = threading.Lock()
    rospy.init_node("new_approach_node", anonymous=True)
    # Subscribers
    rospy.Subscriber("encoder/left/position", Float32, encoder_left_cb)
    rospy.Subscriber("encoder/right/position", Float32, encoder_right_cb)
    # Publishers
    global pos_left_pub, pos_right_pub, rel_pos_left_pub, rel_pos_right_pub, odom_pub
    pos_left_pub = rospy.Publisher("encoder/left/position_unwrapped", Float32, queue_size=1)
    pos_right_pub = rospy.Publisher("encoder/right/position_unwrapped", Float32, queue_size=1)
    rel_pos_left_pub = rospy.Publisher("encoder/left/relative_position", Float32, queue_size=1)
    rel_pos_right_pub = rospy.Publisher("encoder/right/relative_position", Float32, queue_size=1)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
    global odom_msg
    odom_msg = Odometry()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.w = np.cos(yaw * 0.5)
    q.z = np.sin(yaw * 0.5)
    return q

def main():
    init()

    rate = rospy.Rate(50.)
    while not rospy.is_shutdown():

        pX, pY, yaw = odometryClassical()
        # pX, pY, yaw = odometryICC()
        odom_msg.header.seq += 1
        odom_msg.header.stamp = rospy.Time.now()
        position = Point(pX, pY, 0)
        orientation = yaw_to_quaternion(yaw)
        odom_msg.pose.pose = Pose(position, orientation)
        odom_pub.publish(odom_msg)
        rate.sleep()

if __name__ == '__main__':
    main()