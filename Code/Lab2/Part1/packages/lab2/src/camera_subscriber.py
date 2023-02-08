#!/usr/bin/env python3

# Referenced https://github.com/duckietown/sim-duckiebot-lanefollowing-demo/
#   blob/master/custom_line_detector/src/line_detector_node.py

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from duckietown_utils.jpg import bgr_from_jpg
from cv_bridge import CvBridge


class CameraSubscriberNode(DTROS):
    """
    CameraSubscriberNode reads images from the DuckieBot camera, switches the
    blue and green channels, and then publishes these images on a new topic.

    Configuration:
    Subscribers:
        ~/camera_node/image/compressed (:obj: `CompressedImage`)
    Publishers:
        ~/modified_img (:obj: `Image`)
    """
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraSubscriberNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC,
        )

        # Subscribe to the image topic
        rospy.loginfo("Initializing Camera Subscriber Node")
        self.sub = rospy.Subscriber(
            'camera_topic', CompressedImage, self.callback,
        )

        # Publish a modified image to a new topic
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(
            '~modified_img', Image, queue_size=10,
        )

    def callback(self, img_msg):
        # Decode the image data from the incoming message
        data = bgr_from_jpg(img_msg.data)
        rospy.loginfo(f"Image dimensions: {data.shape}")

        # Switch the blue and green channels in the new image data
        b, g = data[:, :, 0], data[:, :, 1]
        new_data = data.copy()
        new_data[:, :, 1] = b
        new_data[:, :, 0] = g

        # Construct the new image message and publish
        img_msg_out = self.bridge.cv2_to_imgmsg(new_data, "bgr8")
        img_msg_out.header.stamp = img_msg.header.stamp
        self.pub.publish(img_msg_out)


if __name__ == '__main__':
    # create the node
    node = CameraSubscriberNode(node_name='camera_subscriber_node')

    # keep spinning
    rospy.spin()
