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

    # Spin in +X axis
    while get_time(t1) < 3:
        controller.add_torque([0.1, -0.1, 0])

    controller.slow_stop()

    t2 = rospy.Time.now()

    # Spin in -Z axis
    while get_time(t2) < 3:
        controller.add_torque([0, 0, -1])
        controller.add_force([0.05, -0.02, 0])
    # Immobilize it
    controller.slow_stop()

    t3 = rospy.Time.now()

    # Apply force/acceleration in 2 axes
    while get_time(t3) < 3:
        controller.add_force([1, 0, -0.2])

    # Immobilize it
    controller.slow_stop()

    rospy.spin()


if __name__ == "__main__":
    main()
