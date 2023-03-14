#!/usr/bin/env python3

# TODO: change from hard-coded vehicle name to using a variable vehicle name

import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.srv import ChangePatternRequest, ChangePattern
from duckietown_msgs.msg import (
    Twist2DStamped,
    LanePose,
    WheelsCmdStamped,
    BoolStamped,
    FSMState,
    StopLineReading,
)
from std_msgs.msg import String, Int8, Empty, Bool
from lane_controller.controller import LaneController
from duckietown_msgs.msg import AprilTagDetectionArray, AprilTagDetection


class IntersectionControllerNode(DTROS):
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(IntersectionControllerNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION,
        )

        self.ff_left = DTParam(
            "~ff_left", param_type=ParamType.FLOAT,
        )
        self.ff_straight = DTParam(
            "~ff_straight", param_type=ParamType.FLOAT,
        )
        self.ff_right = DTParam(
            "~ff_right", param_type=ParamType.FLOAT,
        )
        self.omega_ffs = [
            self.ff_left,
            self.ff_straight,
            self.ff_right,
        ]

        self.time_left_turn = DTParam(
            "~time_left_turn", param_type=ParamType.FLOAT,
        )
        self.time_straight_turn = DTParam(
            "~time_straight_turn", param_type=ParamType.FLOAT,
        )
        self.time_right_turn = DTParam(
            "~time_right_turn", param_type=ParamType.FLOAT,
        )
        self.sleeptimes = [
            self.time_left_turn,
            self.time_straight_turn,
            self.time_right_turn,
        ]

        self.sub_turn_id = rospy.Subscriber(
            "~turn_id", Int8, self._turn,
        )

        rospy.wait_for_service("/csc22939/led_emitter_node/set_pattern")
        self.led_svc = rospy.ServiceProxy(
            "/csc22939/led_emitter_node/set_pattern", ChangePattern,
        )
        self._led_signals = [
            String("CAR_SIGNAL_LEFT"),
            String("CAR_SIGNAL_STRAIGHT"),
            String("CAR_SIGNAL_RIGHT"),
        ]
        self.pub_okay_to_turn = rospy.Publisher(
            "~okay_to_turn", Bool, queue_size=1,
        )
        self.pub_stop_turn = rospy.Publisher(
            "~stop_turn", Empty, queue_size=1,
        )

        self.log("Initialized!")

    def _turn(self, msg):
        turn_id = int(msg.data)

        # Turn on signal light
        msg = ChangePatternRequest(self._led_signals[turn_id])
        self.led_svc(msg)
        rospy.sleep(2)

        # Set the turning speed
        rospy.loginfo(f"=== Setting omega_ff: {self.omega_ffs[turn_id].value}")
        rospy.set_param(
            "/csc22939/lane_controller_node/omega_ff", self.omega_ffs[turn_id].value,
        )
        rospy.sleep(0.5)

        # Signal to lane_controller_node that we can now turn
        self.pub_okay_to_turn.publish(Bool(True))

        # Wait, then reset the turning speed to 0
        rospy.loginfo(f"=== Waiting for {self.sleeptimes[turn_id].value} sec")
        rospy.sleep(self.sleeptimes[turn_id].value)

        rospy.set_param("/csc22939/lane_controller_node/omega_ff", 0)
        rospy.loginfo("=== Setting omega_ff: 0")
        self.pub_stop_turn.publish(Empty())

        # Turn off signal light
        msg = ChangePatternRequest(String("CAR_DRIVING"))
        self.led_svc(msg)

        # Signal to lane_controller_node that turning is complete
        self.pub_okay_to_turn.publish(Bool(False))


if __name__ == "__main__":
    # Initialize the node
    _ = IntersectionControllerNode(node_name="intersection_controller_node")
    # Keep it spinning
    rospy.spin()
