#!/usr/bin/env python3

import pickle
import cv2
import rospy
import rosbag
import numpy as np
import time
from std_msgs.msg import String, Int64, Float64
import pickle
from geometry_msgs.msg import Pose
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import AprilTagDetectionArray, AprilTagDetection


SPIN_OR_RATE = "spin"


class LearnerNode(DTROS):
    def __init__(self, node_name):
        super(LearnerNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION
        )

        self.node_name = node_name
        self._current_tag = -1

        self.previous_state = None
        self.current_state = None
        self._current_state = None
        self.current_state_same_count = 0
        self.current_state_same_count_thresh = DTParam(
            "~current_state_same_count_thresh",
            param_type=ParamType.INT,
            default=3,
        )

        self._lr = rospy.get_param("~learning_rate", None)
        self._tags_used = rospy.get_param("~tags_used", None)

        self._pose = None
        self._town_dims = (1.83, 3.0)  # metres
        self._n = len(self._tags_used)
        self._grid_dims = rospy.get_param("~grid_dims", None)
        self._n_features = np.prod(self._grid_dims)

        # create subscribers
        self._tag_sub = rospy.Subscriber(
            "~detections",
            AprilTagDetectionArray,
            self._tag_cb,
            queue_size=1,
        )

        self.pub = rospy.Subscriber(
            "~odom", Odometry, self._odom_cb, queue_size=1,
        )

        self._switch = rospy.get_param("~switch", False)
        self.srv_set_pattern_ = rospy.Service(
            "~stop_learning", SetBool, self._stop_service,
        )

        self._filename = time.time()

        # Load weights, either from init file or as zeros
        self._weights_init = rospy.get_param("~weights", None)
        if self._weights_init is None:
            self._weights = np.zeros((self._n, self._n_features))
        else:
            with open(self._weights_init, "rb") as infile:
                self._weights = pickle.load(infile)

        # Set up the ros bag, if specified, continue from an existing bag file
        self._bag = rosbag.Bag(f"/data/bags/learner_{self._filename}.bag", "w")
        self._bag_cache = rospy.get_param("~bag_cache", None)
        if self._bag_cache is not None:
            # Populate new rosbag with old data
            self._old_bag = rosbag.Bag(self._bag_cache, "r")
            for topic, msg, _ in self._old_bag.read_messages():
                self._bag.write(topic, msg)
            self._old_bag.close()

    def _stop_service(self, msg):
        self._switch = msg.data

        if not self._switch:
            resp = SetBoolResponse()
            resp.success = True
            resp.message = "learning stopped"
        else:
            resp = SetBoolResponse()
            resp.success = True
            resp.message = "learning started"

        return resp

    def _odom_cb(self, msg):
        self._pose = msg.pose.pose
        # self.previous_state = self.current_state

        current_state = self._get_state()

        # Count the number of times we've seen the current state, only update
        # if we've seen it sufficient times
        if current_state == self._current_state:
            self.current_state_same_count += 1
        else:
            self._current_state = current_state
            self.current_state_same_count = 0

        cont = (self.current_state_same_count <
                self.current_state_same_count_thresh.value)
        if cont:
            return

        if current_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = current_state

            # Predict the apriltag that we are going to see
            gvf_prediction_ind = np.argmax(
                self._weights[:, self.current_state],
            )
            gvf_prediction = self._tags_used[gvf_prediction_ind]
            rospy.loginfo(f"=== Predicting apriltag {gvf_prediction} will " +
                          "be seen next")

            if SPIN_OR_RATE == "spin":
                self.learn()

        # self.current_state = current_state

    def _tag_cb(self, msg):
        if len(msg.detections) == 0:
            self._current_tag = -1
            return
        # elif len(msg.detections) > 1:
        #     raise ValueError("can only update with one apriltag at a time")

        detection = msg.detections[0]

        if detection.tag_id not in self._tags_used:
            return

        rospy.loginfo(f"=== Saw apriltag {detection.tag_id}")

        if detection.tag_id != self._current_tag:
            self._current_tag = detection.tag_id

    def on_shutdown(self):
        rospy.loginfo(f"{self.node_name} received shutdown request")
        with open(f"/data/weights_{self._filename}.pkl", "wb") as outfile:
            pickle.dump(self._weights, outfile)
        self._bag.close()

    def get_tag_index(self, i):
        try:
            return self._tags_used.index(i)
        except KeyError:
            return None

    def _get_state(self):
        pose = (self._pose.position.x, self._pose.position.y)
        return features(pose, self._town_dims, self._grid_dims)

    def _get_reward(self, i):
        if self._current_tag < 0:
            return 0
        return int(i == self._tags_used.index(self._current_tag))

    def learn(self):
        if not self._switch:
            return
        if self.previous_state is None:
            return

        rospy.loginfo("=== learn called ===")
        rospy.loginfo(f"    prev_state: {self.previous_state}")
        rospy.loginfo(f"    curr_state: {self.current_state}")
        rospy.loginfo(f"    curr_tag: {self._current_tag}")

        self._bag.write("curr_tag", Int64(self._current_tag))
        self._bag.write("prev_state", Int64(self.previous_state))
        self._bag.write("curr_state", Int64(self.current_state))

        for i in range(self._n):
            rospy.loginfo(f"    update_weights ({i}) called")
            self.update_weights(i)

        rospy.loginfo("=====================")

    def update_weights(self, i):
        """
        Update the weights for the GVF predictor for apriltag i.
        """
        weights = self._weights[i, :]
        reward = self._get_reward(i)  # cumulant + terminal value

        # Continuation should be 1 until the agent sees an apriltag, in which
        # case the continuation is 0 for all GVFs.
        continuation = self._current_tag < 0

        v = weights[self.previous_state]
        next_v = weights[self.current_state]
        δ = reward + continuation * next_v - v

        weights[self.previous_state] += self._lr * δ

        rospy.loginfo(
            f"            updated weights: {weights}"
        )
        rospy.loginfo(f"            δ: {δ}")
        rospy.loginfo(f"            continuation: {continuation}")

        self._bag.write(f"weights_{i}", String(str(weights)))
        self._bag.write(f"delta_{i}", Float64(δ))
        self._bag.write(f"reward_{i}", Float64(reward))
        self._bag.write(f"continuation_{i}", Float64(continuation))


def bin(input_, metres_dims, grid_dims):
    xi, yi = input_
    x, y = metres_dims
    xgrid, ygrid = grid_dims

    grid_width = x / xgrid
    grid_height = y / ygrid

    return int(xi // grid_width), int(yi // grid_height)


def bin_to_features(bins, grid_dims):
    x, y = bins
    xgrid, _ = grid_dims

    return int(y * xgrid + x)


def features(input_, metres_dims, grid_dims, prev_state=None):
    """
    Return the flattened index of the bin that the bot is currently in
    """
    i = bin_to_features(bin(input_, metres_dims, grid_dims), grid_dims)

    # Project i back to the legal states
    max_ = np.prod(grid_dims) - 1
    min_ = 0

    if i > max_:
        i -= (
            i // grid_dims[0] - grid_dims[1] + 1
        ) * grid_dims[0]
    elif i < min_:
        i += (grid_dims[0] * (np.abs(i // (grid_dims[0]))))

    # Don't allow the state to wrap around the sides of DuckieTown
    if prev_state is not None:
        if prev_state % grid_dims[0] == 0:
            if i < prev_state and i != prev_state - grid_dims[0]:
                i = prev_state
        elif prev_state % (grid_dims[0] - 1) == 0:
            if i > prev_state and i != prev_state + grid_dims[0]:
                i = prev_state
    return i


if __name__ == "__main__":
    rospy.loginfo("learner initializing...")
    learner = LearnerNode("learner_node")
    rospy.loginfo("done!")

    if SPIN_OR_RATE == "spin":
        rospy.spin()
    else:
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            learner.learn()
            rate.sleep()
