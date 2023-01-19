#!/usr/bin/env python3
import cv2
import numpy as np
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


def print_stats(img):
    r, g, b = img[:, :, 0], img[:, :, 1], img[:, :, 2]

    mean_r = np.mean(r)
    median_r = np.median(r)
    mode_r = st.mode(r.ravel()).mode
    max_r = np.max(r)
    min_r = np.min(r)

    mean_g = np.mean(g)
    median_g = np.median(g)
    mode_g = st.mode(g.ravel()).mode
    max_g = np.max(g)
    min_g = np.min(g)

    mean_b = np.mean(b)
    median_b = np.median(b)
    mode_b = st.mode(b.ravel()).mode
    max_b = np.max(b)
    min_b = np.min(b)

    red = format_title("Red")
    green = format_title("Green")
    blue = format_title("Blue")
    lines = (
        f"     {red}   Red   \t{green}Green   {blue}Blue\n\x1b[1;0m",
        "        |------------------------",
        f"Mean    |  {0:3.0f}\t{1:3.0f}\t{2:3.0f}".format(mean_r, mean_g, mean_b),
        f"Median  |  {0:3.0f}\t{1:3.0f}\t{2:3.0f}".format(median_r, median_g, median_b),
        f"Mode    |  {0:3.0f}\t{1:3.0f}\t{2:3.0f}".format(mode_r, mode_g, mode_b),
        f"Max     |  {0:3.0f}\t{1:3.0f}\t{2:3.0f}".format(max_r, max_g, max_b),
        f"Min     |  {0:3.0f}\t{1:3.0f}\t{2:3.0f}".format(min_r, min_g, min_b),
    )

    return lines


def convert_to_range(hist):
    return (
        np.round(
            (hist[0].max() - hist[0]) / (hist[0].max() - hist[0].min()) * MAX,
        ),
        hist[1],
    )


def print_img_data(im, step):
    print("======" * 20)
    for j in range(0, N_SPLITS):
        img = im[j:j+int(step), :, :]
        r, g, b = img[:, :, 0], img[:, :, 1], img[:, :, 2]

        r_hist = np.histogram(r, range=(0, 255), bins=BINS)
        g_hist = np.histogram(g, range=(0, 255), bins=BINS)
        b_hist = np.histogram(b, range=(0, 255), bins=BINS)

        r_hist = convert_to_range(r_hist)
        g_hist = convert_to_range(g_hist)
        b_hist = convert_to_range(b_hist)

        r_lines = print_hist(r_hist, title="Red")
        g_lines = print_hist(g_hist, title="Green")
        b_lines = print_hist(b_hist, title="Blue")
        stats = print_stats(img)

        for i in range(len(r_lines)):
            if i == 0:
                print(f"\x1b[1;33mSplit {j+1}\x1b[1;0m", end="   ")
            else:
                print("      " + " " * len(str(j+1)), end="   ")
            print(r_lines[i], end="    ")
            print(g_lines[i], end="    ")
            if i < len(stats):
                print(b_lines[i], end="    ")
                print(stats[i])
            else:
                print(b_lines[i])
        print()

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

step = None
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("could not get image")
        exit()

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # frame = np.random.randint(0, 255, (w, h, 3))
    # print(frame.shape)
    # exit()

    # Good! I figured it out how to get the image, now the issue is that the
    # histogram is basically 100% for everything...
    frame = cv2.imdecode(frame, 1)
    print(frame.min())
    print(frame.max())
    print(frame.mean())
    with open("/home/duckie/frame2.pkl", "wb") as outfile:
        pickle.dump(frame, outfile)

    # Assuming the frame size will not change, we need to get the step
    if step is None:
        step = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) // N_SPLITS)

        if N_SPLITS > frame.shape[0]:
            shape = frame.shape
            raise ValueError("N_SPLITS should be <= image height but got " +
                             f"N_SPLITS={N_SPLITS} and image shape {shape}")

    # If reading was not successful
    print_img_data(frame, step)

    sleep(1)
