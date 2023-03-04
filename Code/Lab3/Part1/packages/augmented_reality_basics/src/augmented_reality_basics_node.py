#!/usr/bin/env python3

# Adapted from https://github.com/duckietown/dt-core/blob/daffy/packages/ground_projection/src/ground_projection_node.py

from augmented_reality_basics import Augmenter
from copy import deepcopy
import cv2
from cv_bridge import CvBridge
from duckietown_utils.jpg import bgr_from_jpg
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import yaml
from ground_projection_geometry import *  # This class becomes the Augmenter
from rectification import *


class AugmentedRealityBasicsNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(AugmentedRealityBasicsNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC,
        )

        # Get vehicle name
        self.veh = rospy.get_namespace().strip("/")

        # Subscribe to the image topic
        self._sub_image_topic = f"camera_node/image/compressed"
        self._sub = rospy.Subscriber(
            self._sub_image_topic, CompressedImage, self.callback,
        )

        # Get the map file
        self._map_file = rospy.get_param("~map_file")
        rospy.loginfo("map file: " + str(self._map_file))
        rospy.loginfo("cwd: " + str (os.getcwd()))
        self._read_map_file(self._map_file)

        self._aug = Augmenter(self._segments, self._points)

        # Subscribe to camera info
        self._camera_info_received = False
        self.sub_camera_info = rospy.Subscriber(
            "camera_node/camera_info", CameraInfo, self._cb_camera_info, queue_size=1,
        )

        # Publish modified image on a new topic
        self._pub_image_topic = f"~augmented_reality_basics_node/image"
        self._pub = rospy.Publisher(
            self._pub_image_topic, Image, queue_size=1,
        )
        self._bridge = CvBridge()

        # Load in calibration files
        self._read_params_from_calibration_file()

        self.log(f"{node_name} initialized")

    @property
    def points(self):
        return deepcopy(self._points)

    @points.setter
    def points(self, val):
        raise RuntimeError("cannot assign to points")

    def callback(self, img_msg):
        try:
            img = self._aug.render_segments(img_msg)
        except NotImplementedError:
            rospy.signal_shutdown()

        # Publish modified image
        img_out = self._bridge.cv2_to_imgmsg(img, "bgr8")
        img_out.header.stamp = img_msg.header.stamp
        self._pub.publish(img_out)

    def _read_params_from_calibration_file(self):
        """
        Reads the saved parameters from
        `/data/config/calibrations/camera_{intrinsic,extrinsic}/DUCKIEBOTNAME.yaml`
        or uses the default values if the file doesn't exist. Adjsuts the ROS
        parameters for the node with the new values.
        """
        # Get the intrinsic calibration file
        instrinsics_file = "/data/config/calibrations/camera_intrinsic/"
        fname = instrinsics_file + self.veh + ".yaml"

        # Use the default values from the config folder if a robot-specific
        # file does not exist.
        if not os.path.isfile(fname):
            fname = instrinsics_file + "default.yaml"
            self._read_intrinsics_file(fname)
            self.logwarn(f"Intrinsic calibration {fname} not found! " +
                         "Using default.yaml instead.")
        else:
            self._read_intrinsics_file(fname)

        # Get the extrinsic calibration file
        instrinsics_file = "/data/config/calibrations/camera_extrinsic/"
        fname = instrinsics_file + self.veh + ".yaml"

        # Use the default values from the config folder if a robot-specific
        # file does not exist.
        if not os.path.isfile(fname):
            fname = instrinsics_file + "default.yaml"
            self._read_extrinsics_file(fname)
            self.logwarn(f"Extrinsic calibration {fname} not found! " +
                         "Using default.yaml instead.")
        else:
            self._read_extrinsics_file(fname)

    def _read_extrinsics_file(self, fname):
        yaml_dict = self._read_yaml_file(fname)
        self.log(yaml_dict)
        self._homography_mat = np.array(yaml_dict["homography"])
        self._homography_mat = self._homography_mat.reshape(3, 3)

    def _read_intrinsics_file(self, fname):
        yaml_dict = self._read_yaml_file(fname)
        self.log(yaml_dict)
        self._img_w = yaml_dict["image_width"]
        self._img_h = yaml_dict["image_height"]
        self._camera_name = yaml_dict["camera_name"]
        self._camera_mat = yaml_dict["camera_matrix"]
        self._proj_mat = yaml_dict["projection_matrix"]
        self._rect_mat = yaml_dict["rectification_matrix"]
        self._distortion_model = yaml_dict["distortion_model"]
        self._distortion_coef = yaml_dict["distortion_coefficients"]

    def _draw_segment(self, image, pt_x, pt_y, color):
        defined_colors = {
            'red': ['rgb', [1, 0, 0]],
            'green': ['rgb', [0, 1, 0]],
            'blue': ['rgb', [0, 0, 1]],
            'yellow': ['rgb', [1, 1, 0]],
            'magenta': ['rgb', [1, 0, 1]],
            'cyan': ['rgb', [0, 1, 1]],
            'white': ['rgb', [1, 1, 1]],
            'black': ['rgb', [0, 0, 0]]}
        _color_type, [r, g, b] = defined_colors[color]
        rospy.loginfo("drawing from "+ str(pt_x) + " to " + str(pt_y) + " " +
                      str(color))
        cv2.line(
            image,
            (pt_x[0], pt_y[0]),
            (pt_x[1], pt_y[1]),
            (b * 255, g * 255, r * 255),
            5,
        )
        return image

    def _read_map_file(self, fname):
        yaml_map = self._read_yaml_file(fname)
        self._points = yaml_map["points"]
        self._segments = yaml_map["segments"]

    def _read_yaml_file(self, fname):
        """
        Reads the YAML file in the path specified by 'fname'.
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log(f"YAML syntax error. File: {fname} fname. " +
                         f"Exc: {exc}", type="fatal")
                rospy.signal_shutdown(f"could not read file: {fname}")
                return

    def _cb_camera_info(self, msg: CameraInfo):
        """
        Initializes a :py:class:`image_processing.GroundProjectionGeometry`
        object and a :py:class:`image_processing.Rectify` object for image
        rectification

        Args:
            msg (:obj:`sensor_msgs.msg.CameraInfo`): Intrinsic properties of
            the camera.

        References:
         Taken from https://github.com/duckietown/dt-core/blob/daffy/packages/
            ground_projection/src/ground_projection_node.py
        """
        if not self._camera_info_received:
            self._aug.set_rectifier(Rectify(msg))
            self._aug.set_projection_geom(
                    GroundProjectionGeometry(
                    im_width=msg.width,
                    im_height=msg.height,
                    homography=np.array(self._homography_mat).reshape((3, 3)),
                )
            )
        self.camera_info_received = True


if __name__ == '__main__':
    # create the node
    node = AugmentedRealityBasicsNode(
        node_name="augmented_reality_basics_node",
    )

    # keep spinning
    rospy.spin()
