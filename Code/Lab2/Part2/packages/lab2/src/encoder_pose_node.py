#!/usr/bin/env python3

# Adapted from the Duckietown MOOC:
#   https://github.com/duckietown/mooc-exercises/blob/daffy/modcon/solution/src/encoder_pose/src/encoder_pose_node.py
#
# Specifically, I referenced the following file heavily:
#   https://github.com/duckietown/mooc-exercises/blob/daffy/modcon/solution/src/encoder_pose/src/encoder_pose_node.py


from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import (
    Pose2DStamped, Twist2DStamped, WheelEncoderStamped, EpisodeStart,
)
from duckietown_msgs.srv import ChangePattern
from enum import Enum
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Empty
from typing import Optional
import numpy as np
import os
import rosbag
import rospy
import time
import time
import yaml

import odometry_activity


class _State(Enum):
    """
    _State represents one of the four states in the exercise. If the robot is
    in one of these states, it should take the actions of that state as
    outlined in the exercise instructions.
    """
    STOP = 1
    STATE2a = 2
    STATE2b = 5
    STATE2c = 6
    STATE2d = 7
    STATE2e = 8
    STATE2f = 9

    STATE3a = 3
    STATE3b = 10
    STATE3c = 11

    STATE4a = 4
    STATE4b = 12
    STATE4c = 13
    STATE4d = 14


class EncoderPoseNode(DTROS):
    """
    Computes an estimate of the Duckiebot pose using the wheel encoders and
    controls the Duckiebot to perform the actions outlined in the exercise
    instructions.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the ROS node

    Configuration:
    Publisher:
        ~/joy_mapper_node/car_cmd (:obj:`Twist2DStamped`)
        ~shutdown (:obj:`Empty`)
    Subscribers:
        ~/left_wheel_encoder_node/tick (:obj:`WheelEncoderStamped`):
            encoder ticks
        ~/right_wheel_encoder_node/tick (:obj:`WheelEncoderStamped`):
            encoder ticks
    Services used:
        ~/light_service_node/light_change (:obj:`String`)
    """

    right_tick_prev: Optional[int]
    left_tick_prev: Optional[int]
    delta_phi_left: float
    delta_phi_right: float

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(EncoderPoseNode, self).__init__(
            node_name=node_name, node_type=NodeType.CONTROL,
        )
        self._state = None

        self.log(f"Initializing {node_name}...")

        # get the name of the robot
        self.veh = rospy.get_namespace().strip("/")

        # Init the parameters
        self.dist = 0
        self.resetParameters()

        # Record the start time to ensure total run time can be calculated
        # later
        self.start_time = time.time()

        # Write world frame info to a rosbag
        self.world_frame_bag = rosbag.Bag(
            f"/data/bags/world_frame_{self.start_time}.bag", "w",
        )

        # nominal R and L:
        self.log("Loading kinematics calibration...")
        self.R = rospy.get_param(
            f'/{self.veh}/kinematics_node/radius',
            0.318,
        )  # meters, default value of wheel radius
        self.baseline = rospy.get_param(
            f'/{self.veh}/kinematics_node/baseline', 0.1
        )  # meters, default value of baseline

        # Which activities to perform:
        #   ODOMETRY_ACTIVITY:  Calculate the odometry (useful for debugging)
        #   ACTIVITY:           Perform the actions in the exercise
        self.ODOMETRY_ACTIVITY = True
        self.ACTIVITY = True

        # Keep track of info for performing actions in state4
        self._exited_threshold = False
        self._recorded_state4_starting = False

        # #####################################
        # Command publishers
        # #####################################
        # For shutting down nodes
        shutdown_cmd_topic = f"/{self.veh}/shutdown"
        self.pub_shutdown_cmd = rospy.Publisher(
            shutdown_cmd_topic, Empty, queue_size=1,
        )
        # For driving like a car
        car_cmd_topic = f"/{self.veh}/joy_mapper_node/car_cmd"
        self.pub_car_cmd = rospy.Publisher(
            car_cmd_topic,
            Twist2DStamped,
            queue_size=1,
            dt_topic_type=TopicType.CONTROL,
        )

        # Wait until the encoders data is received, then start the controller
        self.duckiebot_is_moving = False
        self.STOP = False

        # For encoders synchronization:
        self.RIGHT_RECEIVED = False
        self.LEFT_RECEIVED = False

        # ####################################
        # Defining subscribers:
        # ####################################
        # Wheel encoder subscriber:
        left_encoder_topic = f"/{self.veh}/left_wheel_encoder_node/tick"
        rospy.Subscriber(
            left_encoder_topic,
            WheelEncoderStamped,
            self.cb_left_encoder,
            queue_size=1,
        )

        # Wheel encoder subscriber:
        right_encoder_topic = f"/{self.veh}/right_wheel_encoder_node/tick"
        rospy.Subscriber(
            right_encoder_topic,
            WheelEncoderStamped,
            self.cb_right_encoder,
            queue_size=1,
        )

        # ########################################
        # Set the service for changing LED colour
        # ########################################
        self._svc_name = f"/{self.veh}/light_service_node/light_change"
        rospy.wait_for_service(self._svc_name)
        self._light_service = rospy.ServiceProxy(self._svc_name, ChangePattern)

        # ########################################
        # Set the colours of different states
        # ########################################
        self._stop_state_light = String("RED")
        self._state2_light = String("GREEN")
        self._state3_light = String("BLUE")
        self._state4_light = String("WHITE")
        self._light_colour = {
            _State.STOP: self._stop_state_light,
            _State.STATE2a: self._state2_light,
            _State.STATE2b: self._state2_light,
            _State.STATE2c: self._state2_light,
            _State.STATE2d: self._state2_light,
            _State.STATE2e: self._state2_light,
            _State.STATE2f: self._state2_light,
            _State.STATE3a: self._state3_light,
            _State.STATE3b: self._state3_light,
            _State.STATE3c: self._state3_light,
            _State.STATE4a: self._state4_light,
            _State.STATE4b: self._state4_light,
            _State.STATE4c: self._state4_light,
            _State.STATE4d: self._state4_light,
        }

        # Set the velocity for moving
        self.steer_gain = 5.5
        self.speed_gain = 0.4

        # Start in the stopped state and wait for 5 seconds, the proceed to
        # state 2a
        self._stop()
        self._set_state(_State.STATE2a)

        self.log(f"{node_name} initialized.")

    def resetParameters(self):
        """
        Resets the parameters at the beginning of the exercise
        """

        # Change in left/right wheel (metres) since previous reading
        self.delta_phi_left = 0.0
        self.delta_phi_right = 0.0

        # Last number of wheel encoder ticks read, needed to compute
        # delta_phi_{left,right}
        self.left_tick_prev = None
        self.right_tick_prev = None

        # Initialize the odometry and total distance travelled
        self.x_prev = 0.32
        self.y_prev = 0.32
        self.theta_prev = np.deg2rad(90)
        self.dist = 0

        self.x_curr = 0.32
        self.y_curr = 0.32
        self.theta_curr = np.deg2rad(90)

    def cb_left_encoder(self, encoder_msg):
        """
        Left wheel encoder callback
        Args:
            encoder_msg (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        # Do nothing if the activity is not set
        if not (self.ODOMETRY_ACTIVITY or self.ACTIVITY):
            return

        # If we have not yet read ticks for the wheel, read them now so that we
        # can calculate φ̇ₗ later
        if self.left_tick_prev is None:
            ticks = encoder_msg.data
            self.left_tick_prev = ticks
            return

        # Calculate the rotation of the left wheel, accumulating previously
        # read rotations. Accumulated values are set to 0 after the pose is
        # calculated.
        delta_phi_left, self.left_tick_prev = odometry_activity.delta_phi(
            encoder_msg, self.left_tick_prev,
        )
        self.delta_phi_left += delta_phi_left

        # Compute the new pose
        self.LEFT_RECEIVED = True
        self.calculate_pose()

    def cb_right_encoder(self, encoder_msg):
        """
        Right wheel encoder callback, the rotation of the wheel.
        Args:
            encoder_msg (:obj:`WheelEncoderStamped`) encoder ROS message.
        """
        # Do nothing if the activity is not set
        if not (self.ODOMETRY_ACTIVITY or self.ACTIVITY):
            return

        # If we have not yet read ticks for the wheel, read them now so that we
        # can calculate φ̇ᵣ later
        if self.right_tick_prev is None:
            ticks = encoder_msg.data
            self.right_tick_prev = ticks
            return

        # Calculate the rotation of the right wheel, accumulating previously
        # read rotations. Accumulated values are set to 0 after the pose is
        # calculated.
        delta_phi_right, self.right_tick_prev = odometry_activity.delta_phi(
            encoder_msg, self.right_tick_prev,
        )
        self.delta_phi_right += delta_phi_right

        # compute the new pose
        self.RIGHT_RECEIVED = True
        self.calculate_pose()

    def calculate_pose(self):
        """
        Calculate the pose of the Duckiebot given by the kinematics model using
        the encoders.
        """
        if self.STOP or not (self.LEFT_RECEIVED and self.RIGHT_RECEIVED):
            return

        # Sync incoming messages from encoders
        self.LEFT_RECEIVED = self.RIGHT_RECEIVED = False

        self.x_curr, self.y_curr, theta_curr, dA = \
            odometry_activity.estimate_pose(
                self.R,
                self.baseline,
                self.x_prev,
                self.y_prev,
                self.theta_prev,
                self.delta_phi_left,
                self.delta_phi_right,
            )

        self.dist += np.abs(dA)

        # Clamp the angle to stay in [0, 2π)
        self.theta_curr = self.angle_clamp(theta_curr)

        # Save world frame data to a rosbag
        pose = Pose2DStamped()
        pose.header.stamp = rospy.Time.now()
        pose.x = self.x_curr
        pose.y = self.y_curr
        pose.theta = self.theta_curr
        self.world_frame_bag.write("world_frame", pose)

        # Log the current angle in degrees
        self.log(f"θ : {np.rad2deg(self.theta_curr)} deg")
        self.log("")

        self.duckiebot_is_moving = (
            abs(self.delta_phi_left) > 0 or
            abs(self.delta_phi_right) > 0
        )

        # Reset the change in wheels to calculate the new odometry only when
        # new data from the encoders arrives. We want to calculate the odometry
        # based on an instant in time (i.e. the change in right/left wheels
        # w.r.t. time φ̇ᵣ and φ̇ₗ).
        self.delta_phi_left = self.delta_phi_right = 0

        # Update our estimation of the pose. Current estimate becomes previous
        # estimate at next iteration.
        self.x_prev = self.x_curr
        self.y_prev = self.y_curr
        self.theta_prev = self.theta_curr

        # Run the controller only in appropriate activities
        if self.ACTIVITY and self._state is not None:
            rospy.loginfo(f"=== State: {self._state}")
            self.Controller()

    def Controller(self):
        """
        Perform the control actions required by the exercise
        """

        time_now = time.time()
        self.time = time_now

        if self._state in (
            _State.STATE2a, _State.STATE2b, _State.STATE2c, _State.STATE2d,
            _State.STATE2e, _State.STATE2f,
        ):
            self._do_state2()
        elif self._state in (_State.STATE3a, _State.STATE3b, _State.STATE3c):
            self._do_state3()
        elif self._state in (
            _State.STATE4a, _State.STATE4b, _State.STATE4c, _State.STATE4d
        ):
            self._do_state4()
        elif self._state == _State.STOP:
            pass
        else:
            rospy.logwarn(f"unknown state {self._state}")
            pass

    def _do_state2(self):
        # Rotate right 90° - to angle 0°
        if self._state == _State.STATE2a:
            deg_threshold = np.deg2rad(15)
            if self.theta_curr > deg_threshold:
                v = -0.2
                omega = -self.steer_gain
                self.publishCmd((v, omega))
            else:
                self.publishCmd((0, 0))
                self._set_state(_State.STATE2b)
            return

        # Drive straight for 1.25 metres
        if self._state == _State.STATE2b:
            dist_threshold = 0.08
            drive = self.dist < 1.25 - dist_threshold
            if drive:
                v, omega = self.speed_gain, -0.2
                if self.theta_curr > 350:
                    omega = 0.1
                elif self.theta_curr > 5:
                    omega = -0.30
                self.publishCmd((v, omega))
            else:
                self.publishCmd((0, 0))
                self._set_state(_State.STATE2c)
            return

        # Rotate left 90° - to angle 90°
        if self._state == _State.STATE2c:
            deg_threshold = np.deg2rad(55)
            if self.theta_curr < deg_threshold:
                v = 0
                omega = self.steer_gain
                self.publishCmd((v, omega))
            else:
                self.publishCmd((0, 0))
                self._set_state(_State.STATE2d)
            return

        # Drive straight for 1.25 metres
        if self._state == _State.STATE2d:
            dist_threshold = 0.25
            drive = self.dist < 2.5 - dist_threshold
            if drive:
                v, omega = self.speed_gain, -0.2
                if self.theta_curr < np.deg2rad(80):
                    omega = 0.1
                elif self.theta_curr > np.deg2rad(90):
                    omega = -0.30
                self.publishCmd((v, omega))
            else:
                self.publishCmd((0, 0))
                self._set_state(_State.STATE2e)
            return

        # Rotate left 90° - to angle 180°
        if self._state == _State.STATE2e:
            deg_threshold = np.deg2rad(140)
            if self.theta_curr < deg_threshold:
                v = 0
                omega = self.steer_gain
                self.publishCmd((v, omega))
            else:
                self.publishCmd((0, 0))
                self._set_state(_State.STATE2f)
            return

        # Drive straight for 1.25 metres
        if self._state == _State.STATE2f:
            dist_threshold = 0.35
            drive = self.dist < 3.75 - dist_threshold
            if drive:
                v, omega = self.speed_gain, -0.17
                if self.theta_curr < np.deg2rad(170):
                    omega = 0.25
                elif self.theta_curr > np.deg2rad(190):
                    omega = -0.25
                self.publishCmd((v, omega))
            else:
                self.publishCmd((0, 0))
                self._stop()
                self._set_state(_State.STATE3a)
            return

    def _do_state3(self):
        # Rotate left 90° - to angle 270°
        if self._state == _State.STATE3a:
            deg_threshold = np.deg2rad(220)
            if self.theta_curr < deg_threshold:
                v = 0.1
                omega = 7.0
                self.publishCmd((v, omega))
            else:
                self.publishCmd((0, 0))
                self._set_state(_State.STATE3b)
            return

        # Drive straight for 1.25 metres
        if self._state == _State.STATE3b:
            dist_threshold = 0.4
            drive = self.dist < 5.0 - dist_threshold
            if drive:
                v, omega = self.speed_gain, -0.30
                if self.theta_curr < np.deg2rad(90):
                    omega = 0.1
                elif self.theta_curr > np.deg2rad(90):
                    omega = -0.45
                self.publishCmd((v, omega))
            else:
                self.publishCmd((0, 0))
                self._set_state(_State.STATE3c)
            return

        # Rotate to 180° in the negative direction - to angle 90°
        if self._state == _State.STATE3c:
            deg_threshold = np.deg2rad(99)
            if self.theta_curr > deg_threshold:
                v = 0.1
                omega = -7.0
                self.publishCmd((v, omega))
            else:
                self.publishCmd((0, 0))
                self._set_state(_State.STATE4a)
            return

    def _do_state4(self):
        if not self._recorded_state4_starting:
            # Record the starting position for state4 so we can try to get back
            # to it
            self._s4_x0 = self.x_curr
            self._s4_y0 = self.y_curr
            self._recorded_state4_starting = True

        # Check whether the Duckiebot is close to the starting (x, y),
        # We consider the Duckiebot "close" to the starting position if it is
        # within circle of radius `xy_thresh` metres of the starting position.
        # Call this the "threshold region".
        xy_thresh = 0.4
        rad = (
            (self.x_curr - self._s4_x0) ** 2 +
            (self.y_curr - self._s4_y0) ** 2
        )
        in_threshold = rad < xy_thresh ** 2

        if not in_threshold and not self._exited_threshold:
            # Record whether we exited the threshold region around the starting
            # (x, y). We exited the threshold if (1) we are currently outside
            # the threshold region and (2) we were just inside the threshold
            # region at the previous time step
            self._exited_threshold = True

        if self._exited_threshold and in_threshold:
            # If we re-entered the threshold region, then stop. We've finished
            # the task!
            self.publishCmd((0, 0))
            total_time = time.time() - self.start_time

            # Log our current position and total execution time
            rospy.loginfo(f"Total execution time: {total_time}")
            rospy.loginfo("Final Location in World Frame:")
            rospy.loginfo(f"\tx: {self.x_curr}")
            rospy.loginfo(f"\ty: {self.y_curr}")
            rospy.loginfo(f"\tθ: {self.theta_curr}")

            self.world_frame_bag.close()

            # Signal shutdown to remaining active nodes
            self.pub_shutdown_cmd.publish(Empty())
            rospy.signal_shutdown("done!")

        # Perform actions for state 4
        elif self._state == _State.STATE4a:
            v, omega = 0.4, 0
            dist_thresh = 0.1
            if self.dist > (1.25 - dist_thresh) / 2:
                self._set_state(_State.STATE4b)
        elif self._state == _State.STATE4b:
            v, omega = 0.4, -2.0
            if 190 < np.rad2deg(self.theta_curr) < 170:
                self._set_state(_State.STATE4c)
        elif self._state == _State.STATE4c:
            v, omega = 0.4, 0

        self.publishCmd((v, omega))

    def publishCmd(self, u):
        """
        Publishes a car command message.
        """
        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()

        car_control_msg.v = u[0]  # v
        car_control_msg.omega = u[1]  # omega

        self.pub_car_cmd.publish(car_control_msg)

    def angle_clamp(self, theta):
        """
        Clamp the input angle `theta` to be in `[0, 2π)`.
        """
        if theta >= 2 * np.pi:
            return theta - 2 * np.pi
        elif theta < 0:  # -2 * np.pi:
            return theta + 2 * np.pi
        else:
            return theta

    def _stop(self):
        """
        Perform the stop state: change LEDs to red and stop for 5 seconds
        """
        self._set_state(_State.STOP)

    def _set_state(self, state: _State):
        """
        Set the state of the robot (i.e. the states described in the exercise
        instructions) and change the LED to the corresponding colour.
        """
        rospy.loginfo(f"=== Setting state {state}")
        self._state = state

        msg = self._light_colour[self._state]
        self._light_service(msg)

        if state == _State.STOP:
            rospy.sleep(5.)


if __name__ == "__main__":
    encoder_pose_node = EncoderPoseNode(node_name="encoder_pose_node")
    rospy.spin()
