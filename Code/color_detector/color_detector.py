#!/usr/bin/env python3
import cv2
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as st
import os
from time import sleep
import pickle
import rospy
from sensor_msgs.msg import CompressedImage, Image
import sys

N_SPLITS = int(os.environ["N_SPLITS"])
BINS = int(os.environ["BINS"])
MAX = int(os.environ["MAX"])
if MAX < 5:
    raise ValueError("MAX should be > 5")


def format_title(title):
    if title == "Red":
        return "\x1b[1;31m"
    elif title == "Green":
        return "\x1b[1;32m"
    elif title == "Blue":
        return "\x1b[1;34m"
    elif title == "Hue":
        return "\x1b[1;33m"
    elif title == "Val":
        return "\x1b[1;35m"
    elif title == "Sat":
        return "\x1b[1;36m"
    else:
        return ""


def print_hist(hist, title=""):
    height, bins = hist
    lines = []

    # Create the title
    length = (len(height) + 1) * 2
    spaces = (length - len(title)) // 2
    rem = length - (spaces * 2 + len(title))
    lines.append(format_title(title) + " " * spaces + title + " " * spaces +
                 " " * rem + "\x1b[1;0m")

    lines.append("⎢" + " " * (length - 2))

    for i in range(MAX, 0, -1):
        line = "⎢"

        for j in range(len(height)):
            if height[j] < i:
                line += "  "
            else:
                line += "█ "
        lines.append(line)

    lines.append("__" * (len(bins) - 1) + "_")
    return lines


def print_hsv_stats(hsv_img):
    h, s, v = hsv_img[:, :, 0], hsv_img[:, :, 1], hsv_img[:, :, 2]

    mean_h = np.mean(h)
    median_h = np.median(h)
    mode_h = st.mode(h.ravel()).mode.item()
    max_h = np.max(h)
    min_h = np.min(h)

    mean_s = np.mean(s)
    median_s = np.median(s)
    mode_s = st.mode(s.ravel()).mode.item()
    max_s = np.max(s)
    min_s = np.min(s)

    mean_v = np.mean(v)
    median_v = np.median(v)
    mode_v = st.mode(v.ravel()).mode.item()
    max_v = np.max(v)
    min_v = np.min(v)

    hue = format_title("Hue")
    sat = format_title("Sat")
    val = format_title("Val")
    lines = (
        f"     {hue}   Hue\t{sat}Sat\t{val}Val\n\x1b[1;0m",
        "        |------------------------",
        f"Mean    |  {mean_h:3.0f}\t{mean_s:3.0f}\t{mean_v:3.0f}",
        f"Median  |  {median_h:3.0f}\t{median_s:3.0f}\t{median_v:3.0f}",
        f"Mode    |  {mode_h:3.0f}\t{mode_s:3.0f}\t{mode_v:3.0f}",
        f"Max     |  {max_h:3.0f}\t{max_s:3.0f}\t{max_v:3.0f}",
        f"Min     |  {min_h:3.0f}\t{min_s:3.0f}\t{min_v:3.0f}",
    )

    return lines, mode_h, mode_s, mode_v


def print_rgb_stats(img):
    b, g, r = img[:, :, 0], img[:, :, 1], img[:, :, 2]

    mean_r = np.mean(r)
    median_r = np.median(r)
    mode_r = st.mode(r.ravel()).mode.item()
    max_r = np.max(r)
    min_r = np.min(r)

    mean_g = np.mean(g)
    median_g = np.median(g)
    mode_g = st.mode(g.ravel()).mode.item()
    max_g = np.max(g)
    min_g = np.min(g)

    mean_b = np.mean(b)
    median_b = np.median(b)
    mode_b = st.mode(b.ravel()).mode.item()
    max_b = np.max(b)
    min_b = np.min(b)

    red = format_title("Red")
    green = format_title("Green")
    blue = format_title("Blue")
    lines = (
        f"     {red}   Red\t{green}Green\t{blue}Blue\t\n\x1b[1;0m",
        "        |------------------------",
        f"Mean    |  {mean_r:3.0f}\t{mean_g:3.0f}\t{mean_b:3.0f}",
        f"Median  |  {median_r:3.0f}\t{median_g:3.0f}\t{median_b:3.0f}",
        f"Mode    |  {mode_r:3.0f}\t{mode_g:3.0f}\t{mode_b:3.0f}",
        f"Max     |  {max_r:3.0f}\t{max_g:3.0f}\t{max_b:3.0f}",
        f"Min     |  {min_r:3.0f}\t{min_g:3.0f}\t{min_b:3.0f}",
    )

    return lines


def convert_to_range(hist):
    return (
        np.round(
            (hist[0].max() - hist[0]) / (hist[0].max() - hist[0].min()) * MAX,
        ),
        hist[1],
    )


# im should be a BGR image
def print_img_data(bgr_im, step):
    hsv_im = cv2.cvtColor(bgr_im, cv2.COLOR_BGR2HSV)
    data = []
    for j in range(0, N_SPLITS):
        bgr_img = bgr_im[j:j+int(step), :, :]
        hsv_img = hsv_im[j:j+int(step), :, :]
        b, g, r = bgr_img[:, :, 0], bgr_img[:, :, 1], bgr_img[:, :, 2]
        h, s, v = hsv_img[:, :, 0], hsv_img[:, :, 1], hsv_img[:, :, 2]

        # For information on colour boundaries, I referenced:
        # https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html
        #   #:~:text=For%20HSV%2C%20hue%20range%20is,need%20to%20normalize
        #   %20these%20ranges.
        r_hist = np.histogram(r, range=(0, 255), bins=BINS)
        g_hist = np.histogram(g, range=(0, 255), bins=BINS)
        b_hist = np.histogram(b, range=(0, 255), bins=BINS)
        h_hist = np.histogram(h, range=(0, 179), bins=BINS)
        s_hist = np.histogram(s, range=(0, 255), bins=BINS)
        v_hist = np.histogram(v, range=(0, 255), bins=BINS)

        r_hist = convert_to_range(r_hist)
        g_hist = convert_to_range(g_hist)
        b_hist = convert_to_range(b_hist)
        h_hist = convert_to_range(h_hist)
        s_hist = convert_to_range(s_hist)
        v_hist = convert_to_range(v_hist)

        r_lines = print_hist(r_hist, title="Red")
        g_lines = print_hist(g_hist, title="Green")
        b_lines = print_hist(b_hist, title="Blue")
        h_lines = print_hist(h_hist, title="Hue")
        s_lines = print_hist(s_hist, title="Sat")
        v_lines = print_hist(v_hist, title="Val")

        rgb_stats = print_rgb_stats(bgr_img)
        hsv_stats, mode_h, mode_s, mode_v = print_hsv_stats(hsv_img)

        # The following code block referenced
        #
        # https://coding-help.fandom.com/wiki/HSV
        #
        # for determining colour names from HSV, taking into account the
        # fact that OpenCV uses H ∈ [0, 180)
        if mode_v < 11:
            colour_name = "\x1b[1;37mblack\x1b[1;0m"
        elif mode_v > 90 and mode_s < 2:
            colour_name = "\x1b[1;37mwhite\x1b[1;0m"
        elif mode_v > 50 and mode_s < 10:
            colour_name = "\x1b[1;37mgray\x1b[1;0m"
        elif mode_s < 15:
            colour_name = "\x1b[1;37mgray\x1b[1;0m"
        else:
            if 0 < mode_h < 30:
                colour_name = "\x1b[1;31mred\x1b[1;0m"
            elif mode_h < 60:
                colour_name = "\x1b[1;33myellow\x1b[1;0m"
            elif mode_h < 90:
                colour_name = "\x1b[1;32mgreen\x1b[1;0m"
            elif mode_h < 120:
                colour_name = "\x1b[1;36mcyan\x1b[1;0m"
            elif mode_h < 150:
                colour_name = "\x1b[1;34mblue\x1b[1;0m"
            else:
                colour_name = "\x1b[1;35mmagenta\x1b[1;0m"

        data.append((r_lines, g_lines, b_lines, h_lines, s_lines, v_lines,
                     rgb_stats, hsv_stats, colour_name))

    os.system('cls' if os.name == 'nt' else 'clear')
    print("======" * 20)
    for j in range(0, N_SPLITS):
        r_lines, g_lines, b_lines, h_lines, s_lines, v_lines, \
            rgb_stats, hsv_stats, colour_name = data[j]

        for i in range(len(r_lines)):
            if i == 0:
                print(f"\x1b[1;33mSplit {j+1}\x1b[1;0m", end="   ")
            else:
                print("      " + " " * len(str(j+1)), end="   ")
            print(r_lines[i], end="    ")
            print(g_lines[i], end="    ")
            if i < len(rgb_stats):
                print(b_lines[i], end="    ")
                print(rgb_stats[i])
            else:
                print(b_lines[i])

        for i in range(len(h_lines)):
            print("      " + " " * len(str(j+1)), end="   ")
            print(h_lines[i], end="    ")
            print(s_lines[i], end="    ")
            if i < len(hsv_stats):
                print(v_lines[i], end="    ")
                print(hsv_stats[i])
            else:
                print(v_lines[i])

        print("The modal hue:       ", mode_h)
        print("The modal saturation:", mode_s)
        print("The modal value:     ", mode_v)
        print("The colour based on these HSV values " +
              f"({mode_h}, {mode_s}, {mode_v}) is {colour_name}")
    print("======" * 20)


def gst_pipeline_string():
    # Parameters from the camera_node
    # Refer here : https://github.com/duckietown/dt-duckiebot-interface/blob/
    #   daffy/packages/camera_driver/config/jetson_nano_camera_node/duckiebot.yaml
    res_w, res_h, fps = 640, 480, 30
    fov = 'full'
    # find best mode
    camera_mode = 3  #
    # compile gst pipeline
    gst_pipeline = f""" \
        nvarguscamerasrc \
        sensor-mode={camera_mode} exposuretimerange="100000 80000000" ! \
        video/x-raw(memory:NVMM), width={res_w}, height={res_h}, format=NV12,
            framerate={fps}/1 ! \
        nvjpegenc ! \
        appsink \
    """

    # ---
    print(f"Using GST pipeline: `{gst_pipeline}`")
    return gst_pipeline


cap = cv2.VideoCapture()
cap.open(gst_pipeline_string(), cv2.CAP_GSTREAMER)

i = 0
step = None
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("could not get image")
        exit(1)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Good! I figured it out how to get the image, now the issue is that the
    # histogram is basically 100% for everything...

    # Referenced the following to figure out image decoding:
    # https://github.com/duckietown/sim-duckiebot-lanefollowing-demo
    bgr_frame = cv2.imdecode(frame, 1)  # BGR colour

    # Assuming the bgr_frame size will not change, we need to get the step only
    # once
    if step is None:
        step = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) // N_SPLITS)

        if N_SPLITS > bgr_frame.shape[0]:
            shape = bgr_frame.shape
            raise ValueError("N_SPLITS should be <= image height but got " +
                             f"N_SPLITS={N_SPLITS} and image shape {shape}")

    # If reading was not successful
    print_img_data(bgr_frame, step)

    sleep(1)
