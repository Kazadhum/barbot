#!/usr/bin/env python3

""" A simple controller for barbot"""

from logging import RootLogger
from pprint import pprint
import random
import time

import numpy as np
import rospy
from webots_ros.msg import Float64Stamped
from webots_ros.srv import get_float, set_float, set_int
from robotController import RobotController


def get_time(t0):
    return (rospy.Time.now() - t0).to_sec()


def get_vel_vec(controller: RobotController):
    vels = controller.get_velocity()

    vel_vec = [
        vels.linear.x,
        vels.linear.y,
        vels.linear.z,
        vels.angular.x,
        vels.angular.y,
        vels.angular.z,
    ]

    return vel_vec


def main():
    """Main control loop."""
    controller = RobotController()

    # Raise it up first
    controller.slow_stop()

    rospy.sleep(duration=5) 

    controller.add_force(force=[0, 0, 0.5], duration=3)
    rospy.sleep(duration=3)
    controller.slow_stop()

    rospy.sleep(duration=5)

    # Spin
    controller.add_torque(torque=[0.1, -0.1, 0.05], duration=3)
    rospy.sleep(duration=3)
    controller.slow_stop()

    rospy.sleep(duration=5)

    for axis in range(3):
        frc = [0.0]*3
        frc[axis] = 0.5

        controller.add_force(force=frc, duration=3)
        rospy.sleep(duration=3)
        controller.slow_stop()
        rospy.sleep(5)

    for axis in range(3):
        trq = [0.0]*3
        trq[axis] = 0.3

        controller.add_torque(torque=trq, duration=3)
        rospy.sleep(duration=3)
        controller.slow_stop()
        rospy.sleep(duration=5)


    # controller.slow_stop()

    # t3 = rospy.Time.now()

    # while get_time(t3) < 3:
    #     controller.add_force([0, -1, 0])

    # controller.slow_stop()

    # t4 = rospy.Time.now()

    # while get_time(t4) < 2:
    #     controller.add_force([0, 0, -0.1])

    # controller.slow_stop()

    # t5 = rospy.Time.now()

    # while get_time(t5) < 2:
    #     controller.add_torque([0.1, 0, 0])

    # controller.slow_stop()

    # t6 = rospy.Time.now()

    # while get_time(t6) < 2:
    #     controller.add_torque([0, 0.1, 0])

    # controller.slow_stop()

    # t7 = rospy.Time.now()

    # while get_time(t7) < 2:
    #     controller.add_torque([0, 0, 0.1])

    # controller.slow_stop()

    rospy.spin()


if __name__ == "__main__":
    main()
