#!/usr/bin/env python3

# Adapted from https://github.com/duckietown/dt-core/blob/daffy/packages/apriltag/src/apriltag_detector_node.py

import numpy as np
import os
import math
import cv2
from renderClass import Renderer

import rospy
import yaml
import sys
from duckietown.dtros import DTROS, NodeType, DTParam, DTReminder
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import rospkg

from sensor_msgs.msg import CameraInfo, CompressedImage
from turbojpeg import TurboJPEG, TJPF_GRAY
from image_geometry import PinholeCameraModel
from dt_apriltags import Detector

"""
This is a template that can be used as a starting point for the CRA1 exercise.
You need to project the model file in the 'models' directory on an AprilTag.
To help you with that, we have provided you with the Renderer class that render the obj file.
"""

class ARNode(DTROS):
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(ARNode, self).__init__(node_name=node_name,node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip("/")

        rospack = rospkg.RosPack()
        # Initialize an instance of Renderer giving the model in input.
        self.renderer = Renderer(rospack.get_path('augmented_reality_apriltag') + '/src/models/duckie.obj')

        # get static parameters
        self.family = rospy.get_param("~family", "tag36h11")
        self.ndetectors = rospy.get_param("~ndetectors", 1)
        self.nthreads = rospy.get_param("~nthreads", 1)
        self.quad_decimate = rospy.get_param("~quad_decimate", 1.0)
        self.quad_sigma = rospy.get_param("~quad_sigma", 0.0)
        self.refine_edges = rospy.get_param("~refine_edges", 1)
        self.decode_sharpening = rospy.get_param("~decode_sharpening", 0.25)
        self.tag_size = rospy.get_param("~tag_size", 0.065)
        self.rectify_alpha = rospy.get_param("~rectify_alpha", 0.0)

        # dynamic parameter
        self.detection_freq = DTParam(
            "~detection_freq", default=-1, param_type=ParamType.INT, min_value=-1, max_value=30
        )
        self._detection_reminder = DTReminder(frequency=self.detection_freq.value)

        # camera info
        self._camera_parameters = None
        self._mapx, self._mapy = None, None

        # create detector object
        self._detectors = [
            Detector(
                families=self.family,
                nthreads=self.nthreads,
                quad_decimate=self.quad_decimate,
                quad_sigma=self.quad_sigma,
                refine_edges=self.refine_edges,
                decode_sharpening=self.decode_sharpening,
            )
            for _ in range(self.ndetectors)
        ]

        self._renderer_busy = False

        # create a CV bridge object
        self._jpeg = TurboJPEG()

        # create subscribers
        self._img_sub = rospy.Subscriber(
            "~image", CompressedImage, self._img_cb, queue_size=1, buff_size="20MB"
        )
        self._cinfo_sub = rospy.Subscriber("~camera_info", CameraInfo, self._cinfo_cb, queue_size=1)

        # create publisher
        self._tag_pub = rospy.Publisher(
            "~detections",
            AprilTagDetectionArray,
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION,
            dt_help="Tag detections",
        )
        self._img_pub = rospy.Publisher(
            "~detections/image/compressed",
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION,
            dt_help="Camera image with tag publishs superimposed",
        )

    def projection_matrix(self, intrinsic, homography):
        """
        Write here the compuatation for the projection matrix, namely the matrix
        that maps the camera reference frame to the AprilTag reference frame.
        """
        # Decode jpeg image to grayscale
        img = self._jpeg.decode(msg.data, pixel_format=TJPF_GRAY)

        # Rectify image -- This should be done with rectification.py
        # img = cv2.remap(img, self._mapx, self._mapy, cv2.INTER_NEAREST)

        tags = self._detectors[detector_id].detect(img, True,
                                                   self._camera_parameters,
                                                   self.tag_size)

        if len(tags) > 0:
            raise NotImplementedError
        rotation_mat = tags[0].pose_R
        transl_mat = tags[0].pose_t
        rospy.loginfo("r: " + str(rotation_mat) + " " + str(type(rotation_mat)))
        rospy.loginfo("t: " + str(transl_mat) + " " + str(type(transl_mat)))

    def readImage(self, msg_image):
        """
            Convert images to OpenCV images
            Args:
                msg_image (:obj:`CompressedImage`) the image from the camera node
            Returns:
                OpenCV image
        """
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg_image)
            return cv_image
        except CvBridgeError as e:
            self.log(e)
            return []

    def readYamlFile(self,fname):
        """
            Reads the 'fname' yaml file and returns a dictionary with its input.

            You will find the calibration files you need in:
            `/data/config/calibrations/`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return


    def onShutdown(self):
        super(ARNode, self).onShutdown()

    def _img_cb(self, msg):
        # make sure we have received camera info
        if self._camera_parameters is None:
            return
        # make sure we have a rectification map available
        if self._mapx is None or self._mapy is None:
            return
        # make sure somebody wants this
        if (not self._img_pub.anybody_listening()) and (not self._tag_pub.anybody_listening()):
            return
        # make sure this is a good time to detect (always keep this as last check)
        if not self._detection_reminder.is_time(frequency=self.detection_freq.value):
            return
        # make sure we are still running
        if self.is_shutdown:
            return

    def _cinfo_cb(self, msg):
        # create mapx and mapy
        H, W = msg.height, msg.width
        # create new camera info
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)

        # find optimal rectified pinhole camera
        with self.profiler("/cb/camera_info/get_optimal_new_camera_matrix"):
            rect_K, _ = cv2.getOptimalNewCameraMatrix(
                self.camera_model.K, self.camera_model.D, (W, H), self.rectify_alpha
            )
            # store new camera parameters
            self._camera_parameters = (rect_K[0, 0], rect_K[1, 1], rect_K[0, 2], rect_K[1, 2])

        # create rectification map
        with self.profiler("/cb/camera_info/init_undistort_rectify_map"):
            self._mapx, self._mapy = cv2.initUndistortRectifyMap(
                self.camera_model.K, self.camera_model.D, None, rect_K, (W, H), cv2.CV_32FC1
            )

        # once we got the camera info, we can stop the subscriber
        self.loginfo("Camera info message received. " +
                     "Unsubscribing from camera_info topic.")
        try:
            self._cinfo_sub.shutdown()
        except BaseException:
            pass

if __name__ == '__main__':
    # Initialize the node
    camera_node = ARNode(node_name='augmented_reality_apriltag_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
