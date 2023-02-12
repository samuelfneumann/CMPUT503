#!/usr/bin/env python3

# Functions in this file are taken and adapted from the Duckietown MOOC:
# https://github.com/duckietown/mooc-exercises/tree/daffy/modcon/solution/04-Odometry

from duckietown_msgs.msg import WheelEncoderStamped
import numpy as np


def delta_phi(encoder_msg: WheelEncoderStamped, prev_ticks):
    """
    Calculate the change in wheel rotation based on the incoming message and
    the previous number of ticks read.
    """
    ticks = encoder_msg.data
    delta_ticks = ticks - prev_ticks
    n_total = encoder_msg.resolution  # ticks per full rotation
    alpha = 2 * np.pi / n_total
    delta_phi = alpha * delta_ticks

    return delta_phi, ticks


def estimate_pose(
    R,
    baseline_wheel2wheel,
    x_prev,
    y_prev,
    theta_prev,
    delta_phi_left,
    delta_phi_right,
):
    """
    Estimate the pose of the Duckiebot

    Parameters
    ----------
    R : float
        The radius of the wheels
    baseline_wheel2wheel : float
        The distance between the two wheels
    x_prev : float
        The previous x position of the Duckiebot
    y_prev : float
        The previous y position of the Duckiebot
    theta_prev : float
        The previous angle of the Duckiebot
    delta_phi_left : float
        The change in the left wheel
    delta_phi_right : float
        The change in the right wheel
    """
    R_left = R_right = R

    d_left = R_left * delta_phi_left
    d_right = R_right * delta_phi_right
    dA = (d_left + d_right) / 2

    dtheta = (d_right - d_left) / baseline_wheel2wheel

    dx = dA * np.cos(theta_prev)
    dy = dA * np.sin(theta_prev)

    # Update pose estimate
    x = x_prev + dx
    y = y_prev + dy
    theta = theta_prev + dtheta

    return x, y, theta, dA
