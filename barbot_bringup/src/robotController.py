#!/usr/bin/env python3

from geometry_msgs.msg import Twist, Vector3
import rospy
import numpy as np
from webots_ros.srv._get_float import get_float
from webots_ros.srv._get_uint64 import get_uint64
from webots_ros.srv._node_add_force_or_torque import node_add_force_or_torque
from webots_ros.srv._node_get_pose import node_get_pose
from webots_ros.srv._node_get_velocity import node_get_velocity
from webots_ros.srv._node_set_velocity import node_set_velocity
from webots_ros.srv._set_int import set_int


class RobotController:
    def __init__(self) -> None:

        rospy.init_node(name="barbot_controller", anonymous=True)

        # Wait for necessary services
        services_to_wait_for = [
            "/supervisor/get_self",
            "/supervisor/node/add_force",
            "/supervisor/node/add_torque",
            "/supervisor/node/get_pose",
        ]

        # Also wait for getBasicTimeStep service
        services_to_wait_for.append("/robot/get_basic_time_step")
        # Additional services
        services_to_wait_for.append("/robot/get_time")

        # sensors = ["accelerometer", "gyro"]

        # for sensor in sensors:
            # services_to_wait_for.append(f"/{sensor}/enable")

        for srv in services_to_wait_for:
            rospy.wait_for_service(service=srv)

        robot_get_basic_time_step_srv = rospy.ServiceProxy(
            name="/robot/get_basic_time_step", service_class=get_float
        )

        supervisor_get_self_srv = rospy.ServiceProxy(
            name="/supervisor/get_self", service_class=get_uint64
        )

        self.supervisor_node = int(supervisor_get_self_srv().value)

        self.basic_time_step = int(robot_get_basic_time_step_srv().value)

        self.srv_proxy_dict = {
            "add_torque": rospy.ServiceProxy(
                name="/supervisor/node/add_torque",
                service_class=node_add_force_or_torque,
            ),
            "add_force": rospy.ServiceProxy(
                name="/supervisor/node/add_force",
                service_class=node_add_force_or_torque,
            ),
            "get_pose": rospy.ServiceProxy(
                name="/supervisor/node/get_pose", service_class=node_get_pose
            ),
            "get_velocity": rospy.ServiceProxy(
                name="/supervisor/node/get_velocity", service_class=node_get_velocity
            ),
            "set_velocity": rospy.ServiceProxy(
                name="/supervisor/node/set_velocity", service_class=node_set_velocity
            ),
        }
        # for sensor in sensors:
        #     self.srv_proxy_dict[f"{sensor}_enable"] = rospy.ServiceProxy(
        #         name=f"/{sensor}/enable", service_class=set_int
        #     )

    # def enable_imu(self):
    #     for sensor in ["accelerometer", "gyro"]:
    #         self.srv_proxy_dict[f"{sensor}_enable"](value=self.basic_time_step)

    def add_torque(self, torque: list):

        torque_vec = Vector3()
        torque_vec.x, torque_vec.y, torque_vec.z = torque

        self.srv_proxy_dict["add_torque"](
            node=self.supervisor_node, force=torque_vec, relative=0
        )

    def add_force(self, force: list):

        force_vec = Vector3()
        force_vec.x, force_vec.y, force_vec.z = force

        self.srv_proxy_dict["add_force"](
            node=self.supervisor_node, force=force_vec, relative=0
        )

    def get_velocity(self):
        vel = self.srv_proxy_dict["get_velocity"](node=self.supervisor_node).velocity
        return vel

    def set_velocity(self, linear: list, angular: list):
        linear_vec = Vector3()
        linear_vec.x, linear_vec.y, linear_vec.z = linear

        angular_vec = Vector3()
        angular_vec.x, angular_vec.y, angular_vec.z = angular

        vel = Twist()
        vel.linear = linear_vec
        vel.angular = angular_vec

        self.srv_proxy_dict["set_velocity"](node=self.supervisor_node, velocity=vel)

    def stop_robot(self):
        self.set_velocity(linear=[0] * 3, angular=[0] * 3)

    def slow_stop(self):

        # First, slow the robot to as close to 0 as possible
        while True:
            # Get velocities first
            vels = self.get_velocity()

            linear_vels = [vels.linear.x, vels.linear.y, vels.linear.z]
            angular_vels = [vels.angular.x, vels.angular.y, vels.angular.z]

            frc: list[float] = [0.0] * 3
            trq: list[float] = [0.0] * 3
            for idx in range(3):
                # apply negative force if speed is positive and vice-versa
                if linear_vels[idx] > 0:
                    frc[idx] = -0.5
                elif linear_vels[idx] < 0:
                    frc[idx] = 0.5

                if angular_vels[idx] > 0:
                    trq[idx] = -0.3
                elif angular_vels[idx] < 0:
                    trq[idx] = 0.3

            # If vels are really close to 0, break
            if (
                np.linalg.norm(linear_vels) < 0.001
                and np.linalg.norm(angular_vels) < 0.001
            ):
                break
            else:
                self.add_force(force=frc)
                self.add_torque(torque=trq)

        # Ensure null velocity
        self.stop_robot()
