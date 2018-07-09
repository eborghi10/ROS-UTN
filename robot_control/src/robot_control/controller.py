#!/usr/bin/env python

class MotorCommand:
    """
    Holds motor control commands for a differential-drive robot.
    """

    def __init__(self):
        self.left = 0
        self.right = 0
        

class Controller:
    """
    Determines motor speeds to accomplish a desired motion.
    """

    def __init__(self):
        # Set the max motor speed to a very large value so that it
        # is, essentially, unbound.
        self.maxMotorSpeed = 10.

    def getSpeeds(self, linearSpeed, angularSpeed):
        tickRate = linearSpeed / self.wheelRadius
        diffTicks = angularSpeed * (self.wheelSeparation/2.) / self.wheelRadius

        speeds = MotorCommand()
        speeds.left = tickRate - diffTicks
        speeds.right = tickRate + diffTicks

        # Adjust speeds if they exceed the maximum.
        if max(speeds.left, speeds.right) > self.maxMotorSpeed:
            factor = self.maxMotorSpeed / max(speeds.left, speeds.right)
            speeds.left *= factor
            speeds.right *= factor

        speeds.left = speeds.left
        speeds.right = speeds.right
        return speeds

    def setWheelSeparation(self, separation):
        self.wheelSeparation = separation

    def setWheelRadius(self, radius):
        self.wheelRadius = radius

    def setMaxMotorSpeed(self, limit):
        self.maxMotorSpeed = limit
