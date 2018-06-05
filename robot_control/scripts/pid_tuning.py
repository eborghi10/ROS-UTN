#!/usr/bin/env python
import rospy
from robot_control import PID
from robot_control.srv import Vel

RATE = 30
SECONDS = 4
SET_POINT_SRV = 'set_point'
MAX_MOTOR_VEL_RPM = 100.
SET_POINT_VELOCITY = 30.

def main():
    idx = 0
    pid = PID(1., 0., 0., encoder_topic='encoder/left', motor_topic='output/left', rate_hz=RATE, set_point_topic=SET_POINT_SRV)
    pid.set_max_motor_vel(pid.to_rad_s(MAX_MOTOR_VEL_RPM))
    rospy.wait_for_service(SET_POINT_SRV)
    set_point_srv_client = rospy.ServiceProxy(SET_POINT_SRV, Vel)
    vel_msg = SET_POINT_VELOCITY
    set_point_srv_client(vel_msg)
    while pid.active():
        idx += 1
        if idx >= RATE * SECONDS:
            idx = 0
            # Service call
            vel_msg = (-1) * vel_msg
            set_point_srv_client(vel_msg)
        pid.calculate()

if __name__ == '__main__':
    main()
