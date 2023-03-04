# Adapted from
# https://github.com/duckietown/dt-core/blob/daffy/packages/complete_image_pipeline/include/image_processing/ground_projection_geometry.py

from typing import NewType, Optional, Tuple

import cv2
import numpy as np


class Point:
    """
    Point class. Convenience class for storing ROS-independent 3D points.
    """
    x: float
    y: float
    z: Optional[float]

    def __init__(self, x=None, y=None, z=None):
        self.x = x  #: x-coordinate
        self.y = y  #: y-coordinate
        self.z = z  #: z-coordinate

    def __repr__(self):
        return f"P({self.x}, {self.y}, {self.z})"

    @staticmethod
    def from_message(msg) -> "Point":
        """
        Generates a class instance from a ROS message. Expects that the message
        has attributes ``x`` and ``y``. If the message additionally has a ``z``
        attribute, it will take it as well. Otherwise ``z`` will be set to 0.

        Args:
            msg: A ROS message or another object with ``x`` and ``y`` attributes

        Returns:
            :py:class:`Point` : A Point object

        """
        x = msg.x
        y = msg.y
        try:
            z = msg.z
        except AttributeError:
            z = 0
        return Point(x, y, z)



ImageSpaceResdepPoint = NewType("ImageSpaceResdepPoint", Point)
ImageSpaceNormalizedPoint = NewType("ImageSpaceNormalizedPoint", Point)
GroundPoint = NewType("GroundPoint", Point)


class GroundProjectionGeometry:
    """
    Handles the Ground Projection operations.

    Note:
        All pixel and image operations in this class assume that the pixels and
        images are *already rectified*. If unrectified pixels or images are
        supplied, the outputs of these operations will be incorrect.

    Args:
        im_width (``int``): Width of the rectified image
        im_height (``int``): Height of the rectified image
        homography (``np.ndarray``): The 3x3 Homography matrix


    """

    im_width: int
    im_height: int
    H: np.ndarray
    Hinv: np.ndarray

    def __init__(self, im_width: int, im_height: int, homography: np.ndarray):

        self.im_width = im_width
        self.im_height = im_height
        H = np.array(homography)
        if H.shape != (3, 3):
            H0 = H
            H = H0.reshape((3, 3))
            rospy.logwarn(f"reshaping your homography matrix:\nfrom\n{H0}\nto\n{H}")

        self.H = H
        self.Hinv = np.linalg.inv(self.H)

    def get_shape(self) -> Tuple[int, int]:
        """returns height, width of image"""
        return self.im_height, self.im_width

    def vector2pixel(self, vec: ImageSpaceNormalizedPoint) -> ImageSpaceResdepPoint:
        """
        Converts a ``[0,1] X [0,1]`` representation to ``[0, W] X [0, H]`` (from
        normalized to image coordinates).

        Args:
            vec (:py:class:`Point`): A :py:class:`Point` object in normalized
            coordinates. Only the ``x`` and ``y`` values are used.

        Returns:
            :py:class:`Point` : A :py:class:`Point` object in image coordinates.
            Only the ``x`` and ``y`` values are used.
        """
        x = self.im_width * vec.x
        y = self.im_height * vec.y
        return ImageSpaceResdepPoint(Point(x, y))

    def pixel2vector(self, pixel: ImageSpaceResdepPoint) -> ImageSpaceNormalizedPoint:
        """
        Converts a ``[0,W] X [0,H]`` representation to ``[0, 1] X [0, 1]`` (from
        image to normalized coordinates).

        Args:
            pixel (:py:class:`Point`): A :py:class:`Point` object in image
            coordinates. Only the ``x`` and ``y`` values are used.

        Returns:
            :py:class:`Point` : A :py:class:`Point` object in normalized
            coordinates. Only the ``x`` and ``y`` values are used.
        """
        x = pixel.x / self.im_width
        y = pixel.y / self.im_height
        return ImageSpaceNormalizedPoint(Point(x, y))

    def pixel2ground(self, pixel: ImageSpaceNormalizedPoint) -> GroundPoint:
        """
        Projects a pixel (``[0, H] X [0, W]``) to the ground
        plane using the homography matrix.

        Args:
            pixel (:py:class:`Point`): A :py:class:`Point` object in
            normalized coordinates. Only the ``x``and ``y`` values are used.

        Returns:
            :py:class:`Point` : A :py:class:`Point` object on the ground plane.
            Only the ``x`` and ``y`` values are used.
        """
        uv_raw = np.array([pixel.x, pixel.y, 1.0])
        ground_point = np.dot(self.H, uv_raw)
        x = ground_point[0]
        y = ground_point[1]
        z = ground_point[2]
        a = x / z
        b = y / z
        return GroundPoint(Point(a, b, 0.0))

    def vector2ground(self, vec: ImageSpaceNormalizedPoint) -> GroundPoint:
        pixel: ImageSpaceResdepPoint = self.vector2pixel(vec)
        return self.pixel2ground(pixel)

    def ground2pixel(self, point: GroundPoint) -> ImageSpaceNormalizedPoint:
        """
        Projects a point on the ground plane to  (``[0, H] X
        [0, W]``) using the homography matrix.

        Args:
            point (:py:class:`Point`): A :py:class:`Point` object on the ground
            plane. Only the ``x`` and ``y`` values are used.

        Returns:
            :py:class:`Point` : A :py:class:`Point` object in normalized
            coordinates. Only the ``x`` and ``y`` values are used.

        Raises:
            ValueError: If the input point's ``z`` attribute is non-zero. The
            point must be on the ground ( ``z=0``).
        """
        if point.z != 0:
            msg = "This method assumes that the point is a ground point (z=0). "
            msg += f"However, the point is ({point.x},{point.y},{point.z})"
            raise ValueError(msg)

        ground_point = np.array([point.x, point.y, 1.0])
        image_point = np.dot(self.Hinv, ground_point)
        image_point = image_point / image_point[2]

        x = image_point[0]
        y = image_point[1]

        return ImageSpaceNormalizedPoint(Point(x, y))
