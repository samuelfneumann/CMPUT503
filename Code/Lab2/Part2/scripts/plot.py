#!/usr/bin/env python3

from matplotlib import animation
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from os.path import expanduser


def animate_theta(i):
    ax.cla()  # clear the previous image
    ax.set_xlim((0, len(theta)))
    ax.set_ylim((-0.2, 2 * np.pi + 0.2))

    ax.set_yticks([0, np.pi/2, np.pi, 3 * np.pi/2, 2 * np.pi])
    ax.set_yticklabels(["0", "$\\frac{\pi}{2}$", "$\pi$", "$\\frac{3\pi}{2}$",
                        "$2\pi$"], fontsize=20)

    ax.plot(theta[:i], linewidth=12, color="blue")
    ax.set_xlabel("$t$", fontsize=20)
    ax.set_ylabel("$\\theta_w$", fontsize=20)

    # Remove labels and ticks along the time axis, which are somewhat
    # uninformative
    plt.tick_params(
        axis="x",
        which="both",
        bottom=False,
        top=False,
        labelbottom=False,
    )

    ax.set_title("Change in $\\theta_w$ over Time $(t)$", fontsize=24)

    ax.axvline(292, linewidth=5, linestyle="--", color="gray", alpha=0.5)


def animate_xy(i):
    ax.cla()  # clear the previous image
    ax.set_xlim((-0.33, 1.67))
    ax.set_ylim((0.20, 2.33))

    ax.plot(x[:i], y[:i], linewidth=12, color="red")
    ax.set_xlabel("$x_w$", fontsize=20)
    ax.set_ylabel("$y_w$", fontsize=20)

    ax.set_xticks([0, 0.5, 1.0, 1.5])
    ax.set_xticklabels(
        ["$0$", "$\\frac{1}{2}$", "$1$", "$\\frac{3}{2}$"], fontsize=20,
    )
    ax.set_yticks([0, 1.0, 2.0])
    ax.set_yticklabels(["$0$", "$1$", "$2$"], fontsize=20)

    ax.set_title("$(x, y)_w$ Position over Time", fontsize=24)
    ax.axvline(
        0.32, linewidth=5, linestyle="--", color="gray", alpha=0.5,
        label="Initial $x_w$",
    )
    ax.axhline(
        0.32, linewidth=5, linestyle="--", color="gray", alpha=0.5,
        label="Initial $y_w$",
    )


def get_rosbag_data(bag_file, topic="/data/bags/world_frame.bag"):
    bag = rosbag.Bag(bag_file, "r")

    x = []
    y = []
    θ = []

    for _, msg, _ in bag.read_messages(topics=[topic]):
        x.append(msg.x)
        y.append(msg.y)
        θ.append(msg.theta)

    return x, y, θ


if __name__ == "__main__":
    x, y, theta = get_rosbag_data("world_frame_1675359838.6775756.bag")

    animate = animate_xy

    fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    anim = animation.FuncAnimation(
        fig,
        animate,
        frames=len(x) + 1,
        interval=1,
        blit=False,
    )

    if animate == animate_xy:
        filename = f"{expanduser('~')}/xy_over_time.gif"
    else:
        filename = f"{expanduser('~')}/theta_over_time.gif"
    writergif = animation.PillowWriter(fps=30)
    writergif.setup(fig, outfile=filename)
    anim.save(filename, writer=writergif)
    plt.show()
