#!/usr/bin/env python3

# Referenced https://github.com/duckietown/sim-duckiebot-lanefollowing-demo/
#   blob/master/custom_line_detector/src/line_detector_node.py

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image
from duckietown_utils.jpg import bgr_from_jpg
from cv_bridge import CvBridge
import numpy as np


class CameraImageNode(DTROS):
    """
    CameraImageNode republishes images from the Duckiebot camera in an
    uncompressed format

    Configuration:
    Subscribers:
        ~/camera_node/image/compressed (:obj: `CompressedImage`)
    Publishers:
        ~/img (:obj: `Image`)
    """
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraImageNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC,
        )

        # Subscribe to the image topic
        rospy.loginfo("Initializing Camera Image Node")
        self.sub = rospy.Subscriber(
            '~camera_topic', CompressedImage, self.callback,
        )

        # Publish a modified image to a new topic
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(
            '~img', Image, queue_size=1,
        )

    def callback(self, img_msg):
        # Decode the image data from the incoming message
        data = bgr_from_jpg(img_msg.data)

        r, g, b = data[:, :, 2], data[:, :, 1], data[:, :, 0]
        new_data = np.zeros_like(data)
        new_data[:, :, 0] = r
        new_data[:, :, 1] = g
        new_data[:, :, 2] = b

        # Construct the new image message and publish
        img_msg_out = self.bridge.cv2_to_imgmsg(new_data, "rgb8")
        self.pub.publish(img_msg_out)


if __name__ == '__main__':
    # create the node
    node = CameraImageNode(node_name='camera_image_node')

    # keep spinning
    rospy.spin()
