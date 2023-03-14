#!/usr/bin/env python3
import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.srv import ChangePatternRequest, ChangePattern
from duckietown_msgs.msg import (
    Twist2DStamped,
    LanePose,
    WheelsCmdStamped,
    BoolStamped,
    StopLineReading,
    VehicleCorners,
)
from std_msgs.msg import String, Int8, Empty, Bool

from lane_controller.controller import LaneController

from duckietown_msgs.msg import AprilTagDetectionArray, AprilTagDetection


class LaneControllerNode(DTROS):
    """Computes control action.
    The node compute the commands in form of linear and angular velocities, by processing the estimate error in
    lateral deviationa and heading.
    The configuration parameters can be changed dynamically while the node is running via ``rosparam set`` commands.
    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use
    Configuration:
        ~v_bar (:obj:`float`): Nominal velocity in m/s
        ~k_d (:obj:`float`): Proportional term for lateral deviation
        ~k_theta (:obj:`float`): Proportional term for heading deviation
        ~k_Id (:obj:`float`): integral term for lateral deviation
        ~k_Iphi (:obj:`float`): integral term for lateral deviation
        ~d_thres (:obj:`float`): Maximum value for lateral error
        ~theta_thres (:obj:`float`): Maximum value for heading error
        ~d_offset (:obj:`float`): Goal offset from center of the lane
        ~integral_bounds (:obj:`dict`): Bounds for integral term
        ~d_resolution (:obj:`float`): Resolution of lateral position estimate
        ~phi_resolution (:obj:`float`): Resolution of heading estimate
        ~omega_ff (:obj:`float`): Feedforward part of controller
        ~verbose (:obj:`bool`): Verbosity level (0,1,2)
        ~stop_line_slowdown (:obj:`dict`): Start and end distances for slowdown at stop lines

    Publisher:
        ~car_cmd (:obj:`Twist2DStamped`): The computed control action
    Subscribers:
        ~lane_pose (:obj:`LanePose`): The lane pose estimate from the lane filter
        ~intersection_navigation_pose (:obj:`LanePose`): The lane pose estimate from intersection navigation
        ~wheels_cmd_executed (:obj:`WheelsCmdStamped`): Confirmation that the control action was executed
        ~stop_line_reading (:obj:`StopLineReading`): Distance from stopline, to reduce speed
        ~obstacle_distance_reading (:obj:`stop_line_reading`): Distancefrom obstacle virtual stopline, to reduce speed
    """

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(LaneControllerNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        # Add the node parameters to the parameters dictionary
        self.params = dict()
        self.params["~v_bar"] = DTParam("~v_bar", param_type=ParamType.FLOAT, min_value=0.0, max_value=5.0)
        self.params["~k_d"] = DTParam("~k_d", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)
        self.params["~k_theta"] = DTParam(
            "~k_theta", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0
        )
        self.params["~k_Id"] = DTParam("~k_Id", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)
        self.params["~k_Dd"] = DTParam("~k_Dd", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0)
        self.params["~k_Iphi"] = DTParam(
            "~k_Iphi", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0
        )
        self.params["~k_Dphi"] = DTParam(
            "~k_Dphi", param_type=ParamType.FLOAT, min_value=-100.0, max_value=100.0
        )
        self.params["~d_offset"] = DTParam(
            "~d_offset",
            param_type=ParamType.FLOAT,
        )

        self.params["~d_thres"] = DTParam(
            "~d_thres",
            param_type=ParamType.FLOAT,
            min_value=0.0,
            max_value=100.0,
        )

        self.params["~theta_thres"] = DTParam(
            "~theta_thres",
            param_type=ParamType.FLOAT,
            min_value=0.0,
            max_value=100.0,
        )
        self.params["~omega_ff"] = DTParam(
            "~omega_ff",
            param_type=ParamType.FLOAT,
            min_value=0.0,
            max_value=8.0,
        )

        self.params["~deriv_type"] = rospy.get_param("~deriv_type", "error")
        self.params["~integral_bounds"] = rospy.get_param("~integral_bounds", None)
        self.params["~d_resolution"] = rospy.get_param("~d_resolution", None)
        self.params["~phi_resolution"] = rospy.get_param("~phi_resolution", None)
        self.params["~verbose"] = rospy.get_param("~verbose", None)
        self.params["~stop_line_slowdown"] = rospy.get_param(
            "~stop_line_slowdown", None,
        )

        # Need to create controller object before updating parameters, otherwise it will fail
        self.controller = LaneController(self.params)
        # self.updateParameters() # TODO: This needs be replaced by the new DTROS callback when it is implemented

        # Initialize variables
        self.wheels_cmd_executed = WheelsCmdStamped()
        self.pose_msg = LanePose()
        self.pose_initialized = False
        self.pose_msg_dict = dict()
        self.last_s = None
        self.stop_line_distance = None
        self.stop_line_detected = False
        self.at_stop_line = False
        self.obstacle_stop_line_distance = None
        self.obstacle_stop_line_detected = False
        self.at_obstacle_stop_line = False
        self.prev_at_stop_line_time = None

        self.current_pose_source = "lane_filter"

        # Construct publishers
        self.pub_car_cmd = rospy.Publisher(
            "~car_cmd", Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        # Construct subscribers
        self.sub_lane_reading = rospy.Subscriber(
            "~lane_pose", LanePose, self.cbAllPoses, "lane_filter", queue_size=1
        )

        # self.sub_intersection_navigation_pose = rospy.Subscriber(
        #     "~intersection_navigation_pose",
        #     LanePose,
        #     self.cbAllPoses,
        #     "intersection_navigation",
        #     queue_size=1,
        # )

        self.sub_wheels_cmd_executed = rospy.Subscriber(
            "~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmdExecuted, queue_size=1
        )
        self.sub_stop_line = rospy.Subscriber(
            "~stop_line_reading", StopLineReading, self.cbStopLineReading, queue_size=1
        )
        self.sub_obstacle_stop_line = rospy.Subscriber(
            "~obstacle_distance_reading", StopLineReading, self.cbObstacleStopLineReading, queue_size=1
        )

        self.prev_veh_avg_x = None
        self.followed_veh_turn = None
        self.sub_centres = rospy.Subscriber(
            "/csc22939/vehicle_detection_node/centers",
            VehicleCorners,
            self._cb_process_veh,
            queue_size=1,
        )

        # Intersection control
        self.possible_turns = [0, 1, 2]  # left, straight, right
        self.tags_to_turns = {
                63: [0, 1],
                59: [1, 2],
                67: [0, 2],
        }

        self.current_tag_id = None
        self.sub_apriltag = rospy.Subscriber(
            "/csc22939/apriltag_detector_node/detections",
            AprilTagDetectionArray,
            self.cbAprilTag,
        )

        self.params["~abs_turning_threshold_camera_frame"] = DTParam(
            "~abs_turning_threshold_camera_frame",
            param_type=ParamType.FLOAT,
            min_value=0.0,
            max_value=640.0,
            default=10.0,
        )

        self._led_signals = [
            String("CAR_SIGNAL_LEFT"),
            String("CAR_SIGNAL_STRAIGHT"),
            String("CAR_SIGNAL_RIGHT"),
        ]
        self.params["~use_LEDs"] = DTParam(
            "~use_LEDs",
            param_type=ParamType.BOOL,
            default=False,
        )

        self.l_turn_v = DTParam(
            "~l_turn_v", default = 0.3,
        )
        self.l_turn_omega = DTParam(
            "~l_turn_omega", default = 1.3,
        )
        self.l_turn_secs = DTParam(
            "~l_turn_secs", default = 1.75,
        )

        self.r_turn_v = DTParam(
            "~r_turn_v", default = 0.3,
        )
        self.r_turn_omega = DTParam(
            "~r_turn_omega", default = -5.0,
        )
        self.r_turn_secs = DTParam(
            "~r_turn_secs", default = 0.6,
        )

        self.s_turn_v = DTParam(
            "~s_turn_v", default = 0.3,
        )
        self.s_turn_omega = DTParam(
            "~s_turn_omega", default = 0.0
        )
        self.s_turn_secs = DTParam(
            "~s_turn_secs", default = 2.5,
        )

        self.turn_params = [
            (self.l_turn_v, self.l_turn_omega, self.l_turn_secs),
            (self.s_turn_v, self.s_turn_omega, self.s_turn_secs),
            (self.r_turn_v, self.r_turn_omega, self.r_turn_secs),
        ]
        self.pose_msg = None

        self.drive_running = False

        # LED control
        rospy.wait_for_service("/csc22939/led_emitter_node/set_pattern")
        self.led_svc = rospy.ServiceProxy(
            "/csc22939/led_emitter_node/set_pattern", ChangePattern,
        )

        if self.params["~use_LEDs"].value:
            msg = ChangePatternRequest(String("WHITE"))
            self.led_svc(msg)
            msg = ChangePatternRequest(String("CAR_DRIVING"))
            msg = ChangePatternRequest(String("CAR_DRIVING"))
            msg = ChangePatternRequest(String("CAR_DRIVING"))
            try:
                resp = self.led_svc(msg)
            except rospy.ServiceException as e:
                rospy.logwarn(f"could not set LEDs: {e}")

        rospy.sleep(2)  # Wait for other nodes to start
        self.log("Initialized!")

    def _available_turns(self):
        avail_turns = self.tags_to_turns.get(self.current_tag_id, [1])
        return avail_turns

    def cbAprilTag(self, msg):
        if len(msg.detections) == 0:
            # No apriltag detected, reset
            self.current_tag_id = None
            return

        for detection in msg.detections:
            if detection.tag_id in self.tags_to_turns.keys():
                self.current_tag_id = detection.tag_id
                return

    def selectTurn(self):
        if self.followed_veh_turn is not None:
            # If following a vehicle, then follow it's turn
            return int(self.followed_veh_turn)
        else:
            # Take a random turn from the available turns (based on apriltag
            # detection)
            turn = self._available_turns()
            return int(np.random.choice(turn))

    def cbObstacleStopLineReading(self, msg):
        """
        Callback storing the current obstacle distance, if detected.

        Args:
            msg(:obj:`StopLineReading`): Message containing information about the virtual obstacle stopline.
        """
        self.obstacle_stop_line_distance = np.sqrt(msg.stop_line_point.x**2 + msg.stop_line_point.y**2)
        self.obstacle_stop_line_detected = msg.stop_line_detected
        self.at_obstacle_stop_line = msg.at_stop_line

    def cbStopLineReading(self, msg):
        """Callback storing current distance to the next stopline, if one is detected.

        Args:
            msg (:obj:`StopLineReading`): Message containing information about the next stop line.
        """
        # Only stop at stop lines at minimum s second intervals
        s = 7
        if msg.at_stop_line:
            if self.prev_at_stop_line_time is not None:
                valid_at_stop_line = (
                    msg.header.stamp.to_sec() - self.prev_at_stop_line_time.to_sec() > s
                )
                if not valid_at_stop_line:
                    return

            self.prev_at_stop_line_time = msg.header.stamp

        self.stop_line_distance = np.sqrt(msg.stop_line_point.x**2 + msg.stop_line_point.y**2)
        self.stop_line_detected = msg.stop_line_detected
        self.at_stop_line = msg.at_stop_line

        # Two other options:
        # 1. Move this logic to drive(); here just store the current stopline
        #    detection and previous one
        # 2. Just increase the time in turns to ensure we get passed the next
        #    stop line

    def cbAllPoses(self, input_pose_msg, pose_source):
        """Callback receiving pose messages from multiple topics.

        If the source of the message corresponds with the current wanted pose source, it computes a control command.

        Args:
            input_pose_msg (:obj:`LanePose`): Message containing information about the current lane pose.
            pose_source (:obj:`String`): Source of the message, specified in the subscriber.
        """

        if pose_source == self.current_pose_source:
            self.pose_msg_dict[pose_source] = input_pose_msg

            self.pose_msg = input_pose_msg

            self.savePose(self.pose_msg)

    def cbWheelsCmdExecuted(self, msg_wheels_cmd):
        """Callback that reports if the requested control action was executed.

        Args:
            msg_wheels_cmd (:obj:`WheelsCmdStamped`): Executed wheel commands
        """
        self.wheels_cmd_executed = msg_wheels_cmd

    def publishCmd(self, car_cmd_msg):
        """Publishes a car command message.

        Args:
            car_cmd_msg (:obj:`Twist2DStamped`): Message containing the requested control action.
        """
        self.pub_car_cmd.publish(car_cmd_msg)

    def savePose(self, pose_msg):
        self.pose_msg = pose_msg

    def cbParametersChanged(self):
        """Updates parameters in the controller object."""
        self.controller.update_parameters(self.params)

    def _cb_process_veh(self, msg):
        # if not self.at_stop_line:
        #     # Can no longer see a vehicle, so reset the previous vehicle's
        #     # position
        #     rospy.loginfo("=== Resetting previous veh")
        #     self.prev_veh_avg_x = None
        #     self.followed_veh_turn = None

        sum_x = 0
        total = len(msg.corners)

        # Compute the average x coordinate over all dots in the dotted pattern
        # on the back of the duckiebot, this will be our estimated x position
        # of the duckiebot in the **camera frame**.
        if total > 0:
            for corner in msg.corners:
                sum_x += corner.x

            avg_x = sum_x / total

            if self.prev_veh_avg_x is None:
                self.prev_veh_avg_x = avg_x
            else:
                # Computations
                # Compute the change in x position, which tells us which
                # direction the followed duckiebot is moving in the camera
                # frame
                delta = avg_x - self.prev_veh_avg_x

                self.prev_veh_avg_x = avg_x

                thresh = self.params["~abs_turning_threshold_camera_frame"].value
                if np.abs(delta) > thresh:
                    self.followed_veh_turn = np.sign(delta) + 1
                else:
                    self.followed_veh_turn = 1

                rospy.loginfo(f"=== Saw turn {self.followed_veh_turn}")

        # else:
        #     # Can no longer see a vehicle, so reset the previous vehicle's
        #     # position
        #     rospy.loginfo("=== Resetting previous veh")
        #     self.prev_veh_avg_x = None
        #     self.followed_veh_turn = None


    def drive(self):
        # IDEA: If at stop line, then turn. Always finish the function with the
        # lane controlling logic though!
        if self.drive_running:
            rospy.logfatal("drive is already running")

        self.drive_running = True

        rospy.loginfo("=== drive called:")
        if self.pose_msg is None:
            self.drive_running = False
            return

        pose_msg = self.pose_msg

        current_s = rospy.Time.now().to_sec()
        dt = None
        if self.last_s is not None:
            dt = current_s - self.last_s

        if self.at_stop_line or self.at_obstacle_stop_line:  # Stop line
            if self.at_obstacle_stop_line:
                rospy.loginfo("at obstacle stop line")
            v = 0
            omega = 0
            car_control_msg = Twist2DStamped()
            car_control_msg.header = pose_msg.header
            self.publishCmd(car_control_msg)

            # Intersection navigation
            if self.at_stop_line and not self.at_obstacle_stop_line:
                rospy.loginfo("At stop line")

                self.at_stop_line = False

                # Set the LED signal lights
                if self.params["~use_LEDs"].value:
                    leds = self._led_signals[turn]
                    rospy.loginfo(f"    Setting leds: {leds}")
                    msg = ChangePatternRequest(leds)
                    try:
                        resp = self.led_svc(msg)
                    except rospy.ServiceException as e:
                        rospy.logwarn(f"could not set LEDs: {e}")

                # Wait at the stop line
                sleep_sec = 3
                rospy.loginfo(f"    Sleeping for {sleep_sec} seconds")
                rospy.sleep(sleep_sec)  # wait at stop line

                # Choose the turn
                turn = int(self.selectTurn())
                rospy.loginfo(f"    Selecting turn: {turn}")

                # Construct turning command
                v, omega, sleep_time = self.turn_params[turn]
                car_control_msg = Twist2DStamped()
                car_control_msg.header.stamp = rospy.Time.now()
                car_control_msg.v = v.value
                car_control_msg.omega = omega.value

                # Turn
                rospy.loginfo(f"    v: {v}, omega: {omega}, time: {sleep_time.value}")
                self.publishCmd(car_control_msg)

                # # Before starting the turn timer, be sure that the robot is
                # # actually moving
                # thresh = 0.01
                # cont = (
                #     np.abs(self.wheels_cmd_executed.vel_left) > thresh and
                #     np.abs(self.wheels_cmd_executed.vel_right) > thresh
                # )
                # while not cont:
                #     rospy.loginfo("    Waiting for turn to begin before timing the turn")
                #     cont = (
                #         np.abs(self.wheels_cmd_executed.vel_left) > thresh and
                #         np.abs(self.wheels_cmd_executed.vel_right) > thresh
                #     )

                rospy.loginfo("    Turning now")
                rospy.sleep(sleep_time.value)

                # Construct the turn stopping command
                car_stop_msg = Twist2DStamped()
                car_stop_msg.v = 0
                car_stop_msg.omega = 0

                # Stop turn
                self.publishCmd(car_stop_msg)
                rospy.loginfo("    Stopping turn")
                # rospy.loginfo(f"        v: {v_stop}, omega: {omega_stop}")

                # Change the LED back to the driving state
                if self.params["~use_LEDs"].value:
                    rospy.loginfo(f"    Changing LEDS back to driving mode")
                    msg = ChangePatternRequest(String("CAR_DRIVING"))
                    try:
                        resp = self.led_svc(msg)
                    except rospy.ServiceException as e:
                        rospy.logwarn(f"could not set LEDs: {e}")

        if not self.at_obstacle_stop_line:  # Lane following
            # Compute errors
            d_err = pose_msg.d - self.params["~d_offset"].value
            phi_err = pose_msg.phi

            # We cap the error if it grows too large
            if np.abs(d_err) > self.params["~d_thres"].value:
                self.log("d_err too large, thresholding it!", "error")
                d_err = np.sign(d_err) * self.params["~d_thres"].value

            wheels_cmd_exec = [self.wheels_cmd_executed.vel_left, self.wheels_cmd_executed.vel_right]
            if self.obstacle_stop_line_detected:
                v, omega = self.controller.compute_control_action(
                    d_err, phi_err, dt, wheels_cmd_exec,
                    self.obstacle_stop_line_distance, pose_msg
                )
                # TODO: This is a temporarily fix to avoid vehicle image detection latency caused unable to stop in time.
                v = v * 0.25
                omega = omega * 0.25

            else:
                v, omega = self.controller.compute_control_action(
                    d_err, phi_err, dt, wheels_cmd_exec,
                    self.stop_line_distance, pose_msg
                )

            # Initialize car control msg, add header from input message
            car_control_msg = Twist2DStamped()
            car_control_msg.header = pose_msg.header

            # Add commands to car message
            car_control_msg.v = v
            car_control_msg.omega = omega

            # self.led_svc(String("CAR_DRIVING"))
            self.publishCmd(car_control_msg)

        # Set the current time stamp, needed for lane following
        # Important: this needs to be set whether we're doing lane following or
        # intersection navigation, otherwise when we go back to lane following
        # from intersection navigation the first step of lane following will
        # break
        self.last_s = current_s
        self.drive_running = False


if __name__ == "__main__":
    # Initialize the node
    rospy.sleep(5)
    node = LaneControllerNode(node_name="lane_controller_node")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.drive()
        rate.sleep()
