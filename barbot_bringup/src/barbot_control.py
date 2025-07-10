#!/usr/bin/env python3

""" A simple controller for the UR5e"""

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


def main():
    """Main control loop."""
    controller = RobotController()

    t0 = rospy.Time.now()

    # Raise it up first
    while get_time(t0) < 3:
        controller.add_force([0, 0, 0.5])

    controller.slow_stop()

    t1 = rospy.Time.now()

    # Spin
    while get_time(t1) < 3:
        controller.add_torque([0.1, -0.1, 0.05])

    controller.slow_stop()

    t2 = rospy.Time.now()

    while get_time(t2) < 3:
        controller.add_force([0.5, 0, 0])

    controller.slow_stop()

    t3 = rospy.Time.now()

    while get_time(t3) < 3:
        controller.add_force([0, -1, 0])

    controller.slow_stop()

    t4 = rospy.Time.now()

    while get_time(t4) < 2:
        controller.add_force([0, 0, -0.1])

    controller.slow_stop()

    t5 = rospy.Time.now()

    while get_time(t5) < 2:
        controller.add_torque([0.1, 0, 0])

    controller.slow_stop()

    t6 = rospy.Time.now()

    while get_time(t6) < 2:
        controller.add_torque([0, 0.1, 0])

    controller.slow_stop()

    t7 = rospy.Time.now()

    while get_time(t7) < 2:
        controller.add_torque([0, 0, 0.1])

    controller.slow_stop()

    rospy.spin()


if __name__ == "__main__":
    main()
