#!/usr/bin/env python
from robot_control import PID

def main():
    MAX_MOTOR_VEL_RPM = 100. # motor maximum velocity (90 rpm)
    pid = PID(1., 0., 0., encoder_topic='encoder/left', motor_topic='output/left')
    pid.set_max_motor_vel(pid.to_rad_s(MAX_MOTOR_VEL_RPM))
    while pid.active():
        pid.calculate()

if __name__ == '__main__':
    main()
