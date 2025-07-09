#!/usr/bin/env python3

from geometry_msgs.msg import Quaternion
import rospy
import tf

from robotController import RobotController


def main():
    controller = RobotController()

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(hz=100)

    while not rospy.is_shutdown():
        robot_pose = controller.get_pose()

        pos = [
            robot_pose.translation.x,
            robot_pose.translation.y,
            robot_pose.translation.z,
        ]

        # For TransformBroadcaster, the quaternion needs to be an array
        quat = [
            robot_pose.rotation.x,
            robot_pose.rotation.y,
            robot_pose.rotation.z,
            robot_pose.rotation.w,
        ]

        br.sendTransform(
            translation=(pos[0], pos[1], pos[2]),
            rotation=quat,
            time=rospy.Time.now(),
            child="base_link",
            parent="world"
        )

        rate.sleep()


if __name__ == "__main__":
    main()
