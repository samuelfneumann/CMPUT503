#!/usr/bin/env python3

# Adapted from
# https://github.com/duckietown/dt-core/blob/daffy/packages/led_emitter/src/led_emitter_node.py

from duckietown.dtros import DTROS, TopicType, NodeType
from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern
from duckietown_msgs.srv import (
    SetCustomLEDPatternResponse, ChangePatternResponse
)
from std_msgs.msg import String, Empty
import rospy
import time


class LightServiceNode(DTROS):
    """
    LightServiceNode implements a service for changing the colours of the the
    Duckiebot LEDs. All LEDs are kept the same colour.

    Services:
        ~/light_service_node/light_change (:obj:`ChangePattern`)
    """
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(LightServiceNode, self).__init__(
            node_name=node_name, node_type=NodeType.DRIVER,
        )

        self.veh_name = rospy.get_namespace().strip("/")

        # Duckiebots already have a service for changing LED colours, use that
        # service to change the colours when users call this node's service
        self._led_service = f"/{self.veh_name}/led_emitter_node/set_pattern"
        rospy.wait_for_service(self._led_service)
        self._embedded_svc = rospy.ServiceProxy(
            self._led_service, ChangePattern,
        )

        # Subscribe to a shutdown notification topic: when we get a message on
        # this topic, shut down the node.
        shutdown_cmd_topic = f"/{self.veh_name}/shutdown"
        rospy.Subscriber(
            shutdown_cmd_topic, Empty, self._shutdown, queue_size=1,
        )

        # Create the service for changing the LED colours
        self._svc = rospy.Service(
            f"{node_name}/light_change",
            ChangePattern,
            self._change_light,
        )

        # Specify the legal colours which the LEDs can be changed to
        self._legal_colours = ["RED", "GREEN", "WHITE", "BLUE", "LIGHT_OFF"]

        self.log("light_service_node Initialized.")

    def _shutdown(self, msg: Empty):
        """
        Performs the shutdown routine of the node
        """
        rospy.signal_shutdown("light_service_node: shutting down...")

    def _change_light(self, colour: ChangePattern):
        """
        Changes the LED colours of the Duckiebot
        """
        if colour.pattern_name.data not in self._legal_colours:
            rospy.loginfo(f"colour should be in {self._legal_colours} " +
                          f"but got {colour}")
        try:
            # Change the colour of the LEDs
            self._embedded_svc(colour.pattern_name)
        except rospy.ServiceException as e:
            rospy.logwarn("exception when changing LED colours: ", + str(e))

        # Reply to the client that we received the request
        return ChangePatternResponse()


if __name__ == "__main__":
    led_emitter_node = LightServiceNode(node_name="light_service_node")
    rospy.spin()
