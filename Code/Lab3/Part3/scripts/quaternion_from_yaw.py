#!/usr/bin/env python3

import numpy as np
import sys
import tf


yaw = float(sys.argv[1])
type_ = "rad" if sys.argv[2].lower() in ("rad", "r", "radians") else "deg"

if type_ == "deg":
    yaw = np.deg2rad(yaw)
    print("yaw in rad: ", yaw)

# q = [
#     np.cos(yaw / 2),
#     0,
#     0,
#     np.sin(yaw / 2),
# ]

q = tf.transformations.quaternion_from_euler(0, 0, yaw)

print("Quaternion:", q, end=" ")
print("{" + f"x: {q[0]}, y: {q[1]}, z: {q[2]}, w: {q[3]}" + "}")

