#!/usr/bin/env python3

import numpy as np
import os
import rosbag
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import (
    Twist2DStamped, Pose2DStamped, WheelEncoderStamped, WheelsCmdStamped,
)
from std_msgs.msg import Header, Float32

# Note: This file was used as a kind of "scratch space" for experimenting with
# driving forward and rotating. Although this code sometimes works, there are
# some bugs. See the Part2 directory for updated code which rotates and drives
# properly

# Referenced:
# https://github.com/duckietown/dt-car-interface/blob/daffy/packages/dagu_car/src/velocity_to_pose_node.py


class OdometryNode(DTROS):
    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """
        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION,
        )
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/radius', 100,
        )
        self._direction = 1
        self._L = 0.10

        self._dr = None
        self._dl = None
        self._first_l = True
        self._first_r = True
        self._last_pose = Pose2DStamped()
        self._n_left_last = None
        self._total_left_dist = 0
        self._n_right_last = None
        self._total_right_dist = 0

        self._task = "rotation"

        # Subscribing to the wheel encoders
        self.ticks_left_topic = \
            f"/{self.veh_name}/left_wheel_encoder_node/tick"
        self.ticks_right_topic = \
            f"/{self.veh_name}/right_wheel_encoder_node/tick"
        self.sub_encoder_ticks_left = rospy.Subscriber(
            self.ticks_left_topic,
            WheelEncoderStamped,
            lambda x: self.cb_encoder_data(self.ticks_left_topic, x),
            dt_topic_type=TopicType.PERCEPTION,
        )
        self.sub_encoder_ticks_right = rospy.Subscriber(
            self.ticks_right_topic,
            WheelEncoderStamped,
            lambda x: self.cb_encoder_data(self.ticks_right_topic, x),
            dt_topic_type=TopicType.PERCEPTION,
        )

        # Publish to topic to control wheels
        self._wheels_cmd_topic = \
            f"/{self.veh_name}/wheels_driver_node/wheels_cmd"
        self._wheels_controller = rospy.Publisher(
            self._wheels_cmd_topic, WheelsCmdStamped, queue_size=10,
        )
        self._car_cmd_topic = f"/{self.veh_name}/car_cmd_switch_node/cmd"
        self._car_cmd_controller = rospy.Publisher(
            self._car_cmd_topic, Twist2DStamped, queue_size=10,
        )

        self.bag = rosbag.Bag("msgs.bag", "w")

        self._pose_topic = f"/{self.veh_name}/velocity_to_pose_node/pose"
        self._pose_sub = rospy.Subscriber(
            self._pose_topic,
            Pose2DStamped,
            self._track_world_coords,
            dt_topic_type=TopicType.PERCEPTION,
        )

        # Publishers
        self.pub_distance_left = rospy.Publisher(
            f"/{self.veh_name}/wheel_dist/left", Float32, queue_size=10
        )
        self.pub_distance_right = rospy.Publisher(
            f"/{self.veh_name}/wheel_dist/right", Float32, queue_size=10,
        )

        self.log("Initialized")
        self._gain = 0.4

    def _track_world_coords(self, msg: Pose2DStamped):
        x, y, θ = msg.x, msg.y, msg.theta

    def _stop(self):
        msg = WheelsCmdStamped()
        msg.vel_left = 0.0
        msg.vel_right = 0.0
        self._wheels_controller.publish(msg)
        rospy.sleep(5.)

    def _drive(self):
        msg = WheelsCmdStamped()
        msg.vel_left = self._gain
        msg.vel_right = msg.vel_left
        self._wheels_controller.publish(msg)

    def _reverse(self):
        cmd = WheelsCmdStamped()
        cmd.vel_left = -self._gain
        cmd.vel_right = cmd.vel_left
        self._wheels_controller.publish(cmd)

    def cb_encoder_data(self, wheel, msg):
        """ Update encoder distance information from ticks.
        """
        # See notes
        # x_t+1 = x_t + Δx where Δx = dA * cos(θ_t)
        # θ_t+1 = θ_t + Δθ where Δθ = (dr - dl) / 2L
        # dA = (dl + dr)/2
        n = np.abs(msg.data)
        n_total = msg.resolution

        after_first_step = self._last_pose.header.stamp.to_sec() > 0
        if wheel == self.ticks_left_topic:
            cont = self._n_left_last is not None and after_first_step
        else:
            cont = self._n_right_last is not None and after_first_step

        if cont:

            # Calculate dl or dr depending on the wheel
            if wheel == self.ticks_left_topic:
                Δwheel = 2 * np.pi * self._radius * (
                    n - self._n_left_last
                ) / n_total
                self._dl = Δwheel
                self._total_left_dist += np.abs(Δwheel)
                self.pub_distance_left.publish(Float32(self._total_left_dist))

            elif wheel == self.ticks_right_topic:
                Δwheel = 2 * np.pi * self._radius * (
                    n - self._n_right_last
                ) / n_total
                self._dr = Δwheel
                self._total_right_dist += np.abs(Δwheel)
                self.pub_distance_right.publish(
                    Float32(self._total_right_dist)
                )

            else:
                raise NotImplementedError()

            if self._dr is not None and self._dl is not None:
                Δθ = (self._dr - self._dl) / (2 * self._L)
                θ_res = self._last_pose.theta + Δθ

                dA = (self._dl + self._dr) / 2
                Δx = dA * np.cos(θ_res)
                x_res = (
                    self._last_pose.x + Δx
                )

                Δy = dA * np.sin(θ_res)
                y_res = (
                    self._last_pose.y + Δy
                )

                self._last_pose.theta = θ_res
                self._last_pose.x = x_res
                self._last_pose.y = y_res

        self._last_pose.header.stamp = msg.header.stamp
        if wheel == self.ticks_left_topic:
            self._n_left_last = n
        elif wheel == self.ticks_right_topic:
            self._n_right_last = n

        if self._task == "rotation":
            if self._last_pose.theta > -np.pi / 2:
                rospy.loginfo("θ: " + str(self._last_pose.theta))
                cmd = Twist2DStamped()
                cmd.omega = -4
                cmd.v = 0
                self._car_cmd_controller.publish(cmd)
            else:
                cmd = Twist2DStamped()
                cmd.v = 0
                cmd.omega = 0
                self._car_cmd_controller.publish(cmd)
        else:
            return

        if self._direction > 0:
            x = self._last_pose.x
            y = self._last_pose.y
            drive = self._total_right_dist < 1.25 and \
                self._total_left_dist < 1.25
            if drive:
                self._drive()
            else:
                self._final_length = x ** 2 + y ** 2
                self._stop()
                self._direction = -1
        else:
            x = self._last_pose.x
            y = self._last_pose.y
            drive = self._total_right_dist < 2.5 and \
                self._total_left_dist < 2.5
            if drive:
                self._reverse()
            else:
                self._stop()

        self.bag.write(wheel, msg)

        return


if __name__ == '__main__':
    node = OdometryNode(node_name=f'odometry_node')

    rospy.spin()

    node.bag.close()
