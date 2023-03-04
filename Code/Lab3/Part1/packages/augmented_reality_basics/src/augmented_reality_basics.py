from copy import deepcopy
import cv2
from duckietown_utils.jpg import bgr_from_jpg
import numpy as np
from sensor_msgs.msg import CompressedImage
import rospy
from ground_projection_geometry import *  # This class becomes the Augmenter
from rectification import *


class Augmenter:
    """
    This class assumes all data/pixel coords and images have been rectified.

    The input image, read from the camera topic, is first rectified before
    plotting any lines on it. We assume the coordinates of the lines which have
    been given to us are already in the rectified coordinate system.
    """
    def __init__(self, segments, points):
        self._segments = segments
        self._points = points
        self.set_projection_geom(None)
        self.set_rectifier(None)

    def set_projection_geom(self, geom):
        self._geom = geom

    def set_rectifier(self, r):
        self._rectifier = r

    @property
    def points(self):
        return deepcopy(self._points)

    def process_image(self, img_msg: CompressedImage):
        return self._rectifier.rectify(bgr_from_jpg(img_msg.data))

    def _ground2pixel(self, coords: np.array):
        # The ground coordinates are with respect to the robot frame.
        #
        # x is forward/backward, positive is forward
        # y is side to side from robot, positive to the left
        x = coords[0]
        y = coords[1]
        z = 0
        pt = Point(x, y, z)

        pixel = self._geom.ground2pixel(pt)

        return int(pixel.y), int(pixel.x)

    def render_segments(self, img_msg: CompressedImage):
        # Get image data from image message
        img = self.process_image(img_msg)
        h, w, _ = img.shape

        # Draw lines
        for segment in self._segments:
            # Ensure the coordinate types are valid
            coord_type_1 = self.points[segment["points"][0]][0]
            coord_type_2 = self.points[segment["points"][1]][0]

            # Convert to image-plane coordinates
            pt_1 = list(self.points[segment["points"][0]][1])
            if coord_type_1 == "image01":
                pt_1 = self._img2camera(pt_1)
            elif coord_type_1 == "axle":
                pt_1 = self._ground2pixel(pt_1)
            elif coord_type_1 == "camera":
                pt_1 = self._camera2camera(pt_1)
            else:
                raise NotImplementedError()
                rospy.signal_shutdown()

            pt_2 = list(self.points[segment["points"][1]][1])
            if coord_type_2 == "image01":
                pt_2 = self._img2camera(pt_2)
            elif coord_type_2 == "axle":
                pt_2 = self._ground2pixel(pt_2)
            elif coord_type_2 == "camera":
                pt_2 = self._camera2camera(pt_2)
            else:
                raise NotImplementedError()
                rospy.signal_shutdown()

            pt_x = np.array([pt_1[1], pt_2[1]])
            pt_y = np.array([pt_1[0], pt_2[0]])

            # Draw line
            colour = segment["color"]
            img = self._draw_segment(img, pt_x, pt_y, colour)

        return img

    def _camera2camera(self, coords):
        y = coords[0]
        x = coords[1]
        pixel = Point(x, y, 0)

        return int(pixel.y), int(pixel.x)

    def _img2camera(self, coords):
        """
        Convert a point in image space ([0, 1] × [0, 1]) to camera space ([0,
        H] × [0, W])
        """
        y = coords[0]
        x = coords[1]
        pt = Point(x, y)

        pixel = self._geom.vector2pixel(pt)
        return int(pixel.y), int(pixel.x)

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
