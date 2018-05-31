#!/usr/bin/env python
# Import dependencies
import math
import rospy
from std_msgs.msg import Float32, Int16
from robot_control.srv import Vel, VelResponse
from dynamic_reconfigure.server import Server
from robot_control.cfg import PidParametersConfig

class PID:
# https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_PID
# TODO: Use rosparam for some constants and dynamic reconfigure
    '''
    PID class for doing a DC Motor velocity control
    '''
    def __init__(self, kP, kI, kD, motor_max_vel=10, target_velocity=0, rate_hz=30, i_clamp=10, alpha=1.0 \
        encoder_topic='encoder', motor_topic='output', set_point_topic='set_point', error_topic='error'):
        # PID parameters
        self.kP = float(kP)
        self.kI = float(kI)
        self.kD = float(kD)
        self.MAX_MOTOR_VEL_M_S = float(motor_max_vel)
        self.target_velocity = float(target_velocity)
        self.previous_error = 0.
        self.previous_time = 0.
        self.error_sum = 0.
        self.i_clamp = i_clamp
        # Set alpha for derivative filter smoothing factor
        self.alpha = float(alpha)
        # Set publishers and subscribers
        rospy.init_node('pid_node')
        self.rate = rospy.Rate(rate_hz)
        rospy.on_shutdown(self.__on_shutdown_cb__)

        rospy.Subscriber(encoder_topic, Float32, self.__encoder_cb__)
        rospy.Service(set_point_topic, Vel, self.set_point_srv)
        Server(PidParametersConfig, self.__update_pid_parameters__)
        self.motor_pub = rospy.Publisher(motor_topic, Int16, queue_size=10, latch=True)
        self.error_pub = rospy.Publisher(error_topic, Float32, queue_size=10, latch=True)

        self.encoder_msg = 0.

    def __get_velocity__(self):
        error = self.__get_error__()
        self.error_pub.publish(error)
        return error

    def __get_error__(self):
        current_time = rospy.get_time()
        dT = current_time - self.previous_time
        if dT == 0:
            return 0
        current_error = self.target_velocity - self.encoder_msg
        # Proportional
        error_P = current_error
        # Address max windup
        if self.error_sum > self.i_clamp:
            self.error_sum = self.i_clamp
        elif self.error_sum < -self.i_clamp:
            self.error_sum = -self.i_clamp
        # Integral
        self.error_sum += (current_error * dT)
        # TODO: USE slef.i_clamp to trim
        error_I = self.error_sum
        # Derivative
        delta_error = current_error - self.previous_error
        # Recalculate the derivative error here incorporating derivative smoothing!
        error_D = self.alpha * delta_error / dT + (1 - self.alpha)
        # Sum all errors
        error = self.kP * error_P + self.kI * error_I + self.kD * error_D
        self.previous_error = current_error
        self.previous_time = current_time
        return error

    def publish_velocity(self, error_vel):
        motor_vel = self.__to_motor__(error_vel)
        self.motor_pub.publish(motor_vel)
        self.rate.sleep()

    def control_loop(self):
        while not rospy.is_shutdown():
            velocity = self.__get_velocity__()
            self.publish_velocity(velocity)

    def __encoder_cb__(self, data):
        self.encoder_msg = data.data

    def set_point_srv(self, req):
        self.target_velocity = float(req.velocity)
        return VelResponse()

    def __on_shutdown_cb__(self):
        rospy.loginfo("Stopping motor")
        self.motor_pub.publish(0)

    def to_rad_s(self, rpm):
        return rpm / 9.5492965964254

    def __to_motor__(self, rad_s):
        '''
        Converts velocity from rad/s to [0;255]
        '''
        # Enforce actuator saturation limits
        if rad_s > self.MAX_MOTOR_VEL_M_S:
            rad_s = self.MAX_MOTOR_VEL_M_S
        elif rad_s < -self.MAX_MOTOR_VEL_M_S:
            rad_s = -self.MAX_MOTOR_VEL_M_S

        return rad_s * 255. / self.MAX_MOTOR_VEL_M_S

    def set_max_motor_vel(self, velocity):
        self.MAX_MOTOR_VEL_M_S = velocity

    def get_max_motor_vel(self):
        return self.MAX_MOTOR_VEL_M_S

    def __update_pid_parameters__(self, config, level):
        rospy.loginfo("""Reconfigure Request: {p}, {i}, {d}, {i_clamp}""".format(**config))
        self.kP = config['p']
        self.kI = config['i']
        self.kD = config['i']
        self.i_clamp = config['i_clamp']
        return config

    def reset(self):
        self.target_velocity = 0.
        self.kP = 0.
        self.kI = 0.
        self.kD = 0.
        self.error_sum = 0.
        self.i_clamp = 0.
        self.previous_time = 0.
        self.previous_error = 0.

def main():
    MAX_MOTOR_VEL_RPM = 100. # motor maximum velocity (90 rpm)
    pid = PID(1., 0., 0., encoder_topic='encoder/left', motor_topic='output/left')
    pid.set_max_motor_vel(pid.to_rad_s(MAX_MOTOR_VEL_RPM))
    pid.control_loop()

if __name__ == '__main__':
    main()
