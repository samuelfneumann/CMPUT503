#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String


class MyPublisherNode(DTROS):
    """
    MyPublisherNode publishes welcome messages on the `~chatter` topic
    """
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC,
        )

        # construct publisher
        self.pub = rospy.Publisher('~chatter', String, queue_size=10)

    def run(self):
        """
        Runs the publisher node by publishing a message on the ~chatter topic
        at a frequency of 1Hz.
        """
        # publish message every 1 second
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            message = "Hello from %s" % os.environ['VEHICLE_NAME']
            rospy.loginfo("Publishing message: '%s'" % message)
            self.pub.publish(message)
            rate.sleep()


if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')

    # run node
    node.run()

    # keep spinning
    rospy.spin()
