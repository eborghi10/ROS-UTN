#!/usr/bin/env python
# Import dependencies
import rospy
from std_msgs.msg import Float32, Int16

class PID:
# https://syrotek.felk.cvut.cz/course/ROS_CPP_INTRO/exercise/ROS_CPP_PID
# TODO: Use rosparam for some constants and dynamic reconfigure
    '''
    PID class for doing a DC Motor velocity control
    '''
    def __init__(self, kP, kI, kD, motor_max_vel=10., target_velocity=0., rate_hz=50., \
        encoder_topic='encoder', motor_topic='output', set_point_topic='set_point', error_topic='error'):
        # PID parameters
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.MAX_MOTOR_VEL_M_S = motor_max_vel
        self.target_velocity = target_velocity
        # Set publishers and subscribers
        rospy.init_node('pid_node', anonymous=True)
        self.rate = rospy.Rate(rate_hz)
        rospy.on_shutdown(self.__on_shutdown_cb__)

        rospy.Subscriber(encoder_topic, Float32, self.__encoder_cb__)
        rospy.Service(set_point_topic, , self.__set_point_srv__)
        self.motor_pub = rospy.Publisher(motor_topic, Int16, queue_size=10, latch=True)
        self.error_pub = rospy.Publisher(error_topic, Float32, queue_size=10, latch=True)

        self.encoder_msg = 0.

    def __get_velocity__(self):
        error = self.__get_error__()
        self.error_pub.publish(error)
        return error
    
    def __get_error__(self):
        error = self.target_velocity - self.encoder_msg
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
    
    def __set_point_srv__(self, req):
        self.target_velocity = req.vel
        return std_srvs.Empty()

    def __on_shutdown_cb__(self):
        rospy.loginfo("Stopping motor")
        self.motor_pub.publish(0)

    def to_rad_s(self, rpm):
        return rpm / 9.5492965964254

    def __to_motor__(self, rad_s):
        return rad_s * 255. / self.MAX_MOTOR_VEL_M_S
    
    def set_max_motor_vel(self, velocity):
        self.MAX_MOTOR_VEL_M_S = velocity

    def get_max_motor_vel(self):
        return self.MAX_MOTOR_VEL_M_S

def main():
    MAX_MOTOR_VEL_RPM = 90 # motor maximum velocity (90 rpm)
    pid = PID(1., 0., 0.)
    pid.set_max_motor_vel(pid.to_rad_s(MAX_MOTOR_VEL_RPM))
    pid.control_loop()

if __name__ == '__main__':
    main()
