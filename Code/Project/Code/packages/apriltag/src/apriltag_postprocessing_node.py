#!/usr/bin/env python3
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import (
    AprilTagDetection,
    AprilTagDetectionArray,
    AprilTagsWithInfos,
    BoolStamped,
    TagInfo,
)
from duckietown_msgs.srv import ChangePattern, ChangePatternRequest
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage
import tf2_ros
import tf2_geometry_msgs  # For transforming PoseStamped
import tf

from geometry_msgs.msg import Quaternion, Twist, Pose, Point, Vector3, TransformStamped, Transform


# TODO: should this inherit DTROS?
class AprilPostPros(object):
    """ """

    def __init__(self):
        """ """
        self.node_name = "apriltag_postprocessing_node"

        self._tf_bcaster = tf.TransformBroadcaster()
        # Load parameters
        self.camera_x = self.setup_param("~camera_x", 0.065)
        self.camera_y = self.setup_param("~camera_y", 0.0)
        self.camera_z = self.setup_param("~camera_z", 0.11)
        self.camera_theta = self.setup_param("~camera_theta", 19.0)
        self.scale_x = self.setup_param("~scale_x", 1)
        self.scale_y = self.setup_param("~scale_y", 1)
        self.scale_z = self.setup_param("~scale_z", 1)

        # -------- Start adding back the tag info stuff
        tags_filepath = self.setup_param("~tags_file")

        self.loc = self.setup_param("~loc", -1)  # -1 if no location is given
        tags_file = open(tags_filepath, "r")
        self.tags_dict = yaml.safe_load(tags_file)
        tags_file.close()
        self.info = TagInfo()

        self.sign_types = {
            "StreetName": self.info.S_NAME,
            "TrafficSign": self.info.SIGN,
            "Light": self.info.LIGHT,
            "Localization": self.info.LOCALIZE,
            "Vehicle": self.info.VEHICLE,
        }
        self.traffic_sign_types = {
            "stop": self.info.STOP,
            "yield": self.info.YIELD,
            "no-right-turn": self.info.NO_RIGHT_TURN,
            "no-left-turn": self.info.NO_LEFT_TURN,
            "oneway-right": self.info.ONEWAY_RIGHT,
            "oneway-left": self.info.ONEWAY_LEFT,
            "4-way-intersect": self.info.FOUR_WAY,
            "right-T-intersect": self.info.RIGHT_T_INTERSECT,
            "left-T-intersect": self.info.LEFT_T_INTERSECT,
            "T-intersection": self.info.T_INTERSECTION,
            "do-not-enter": self.info.DO_NOT_ENTER,
            "pedestrian": self.info.PEDESTRIAN,
            "t-light-ahead": self.info.T_LIGHT_AHEAD,
            "duck-crossing": self.info.DUCK_CROSSING,
            "parking": self.info.PARKING,
            None: None,
        }
        # ---- end tag info stuff

        self.sub_prePros = rospy.Subscriber(
            "apriltag_detector_node/detections", AprilTagDetectionArray, self.callback, queue_size=1
        )
        self.pub_postPros = rospy.Publisher("~apriltags_out", AprilTagsWithInfos, queue_size=1)
        # self.pub_visualize = rospy.Publisher("~tag_pose", PoseStamped, queue_size=1)

        # topics for state machine
        self.pub_postPros_parking = rospy.Publisher(
            "~apriltags_parking", BoolStamped, queue_size=1,
        )
        self.pub_postPros_intersection = rospy.Publisher(
            "~apriltags_intersection", BoolStamped, queue_size=1,
        )

        # # Service for changing LED colours
        # self._led_svc = rospy.ServiceProxy(
        #     "led_emitter_node/set_pattern", ChangePattern,
        # )

        self.static_frames = {}
        self.sub_static = rospy.Subscriber(
            "/tf_static", TFMessage, self.static_cb, queue_size=1
        )

        self.update_odom = rospy.Publisher(
            "deadreckoning_node/update_odom", Pose, queue_size=1,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def static_cb(self, msg):
        for transf in msg.transforms:
            if transf.header.frame_id != "world":
                continue
            self.static_frames[transf.child_frame_id] = transf

    def setup_param(self, param_name, default_value=rospy.client._Unspecified):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)
        return value

    def _switch_led(self, id_info):
        sign = self.traffic_sign_types[id_info["traffic_sign_type"]]
        sign_types = self.traffic_sign_types
        t_intersect_signs = set(
            (
                sign_types["right-T-intersect"],
                sign_types["left-T-intersect"],
                sign_types["T-intersection"],
            )
        )

        # if sign == sign_types["stop"]:
        #     self._led_svc(String("RED"))
        # elif sign in t_intersect_signs:
        #     self._led_svc(String("BLUE"))
        # elif sign == sign_types["4-way-intersect"]:
        #     self._led_svc(String("GREEN"))
        # elif sign is sign_types[None]:  # None
        #     self._led_svc(String("WHITE"))
        # else:
        #     raise RuntimeError(f"unknown traffic sign {sign}")

    def callback(self, msg):
        tag_infos = []

        new_tag_data = AprilTagsWithInfos()

        id_info = {
            'tag_id': None,
            'tag_type': None,
            'street_name': None,
            'vehicle_name': None,
            'traffic_sign_type': None,
        }

        # Load tag detections message
        for detection in msg.detections:
            # ------ start tag info processing
            new_info = TagInfo()
            # Can use id 1 as long as no bundles are used
            new_info.id = int(detection.tag_id)
            id_info = self.tags_dict[new_info.id]

            # Check yaml file to fill in ID-specific information
            new_info.tag_type = self.sign_types[id_info["tag_type"]]

            if new_info.tag_type == self.info.S_NAME:
                new_info.street_name = id_info["street_name"]
            elif new_info.tag_type == self.info.SIGN:
                new_info.traffic_sign_type = self.traffic_sign_types[id_info["traffic_sign_type"]]

                # publish for FSM
                # parking apriltag event
                msg_parking = BoolStamped()
                msg_parking.header.stamp = rospy.Time(0)
                if new_info.traffic_sign_type == TagInfo.PARKING:
                    msg_parking.data = True
                else:
                    msg_parking.data = False
                self.pub_postPros_parking.publish(msg_parking)

                # intersection apriltag event
                msg_intersection = BoolStamped()
                msg_intersection.header.stamp = rospy.Time(0)
                if (
                    (new_info.traffic_sign_type == TagInfo.FOUR_WAY)
                    or (new_info.traffic_sign_type == TagInfo.RIGHT_T_INTERSECT)
                    or (new_info.traffic_sign_type == TagInfo.LEFT_T_INTERSECT)
                    or (new_info.traffic_sign_type == TagInfo.T_INTERSECTION)
                ):
                    msg_intersection.data = True
                else:
                    msg_intersection.data = False
                self.pub_postPros_intersection.publish(msg_intersection)

            elif new_info.tag_type == self.info.VEHICLE:
                new_info.vehicle_name = id_info["vehicle_name"]

            if self.loc == 226:
                l = id_info["location_226"]
                if l is not None:
                    new_info.location = l
            elif self.loc == 316:
                l = id_info["location_316"]
                if l is not None:
                    new_info.location = l

            tag_infos.append(new_info)
            # --- end tag info processing

            # Define the transforms
            veh_t_camxout = tr.translation_matrix((self.camera_x, self.camera_y, self.camera_z))
            veh_R_camxout = tr.euler_matrix(0, self.camera_theta * np.pi / 180, 0, "rxyz")
            veh_T_camxout = tr.concatenate_matrices(
                veh_t_camxout, veh_R_camxout
            )  # 4x4 Homogeneous Transform Matrix

            camxout_T_camzout = tr.euler_matrix(-np.pi / 2, 0, -np.pi / 2, "rzyx")
            veh_T_camzout = tr.concatenate_matrices(veh_T_camxout, camxout_T_camzout)

            tagzout_T_tagxout = tr.euler_matrix(-np.pi / 2, 0, np.pi / 2, "rxyz")

            # Load translation
            trans = detection.transform.translation
            rot = detection.transform.rotation
            header = Header()
            header.stamp = rospy.Time.now()
            camzout_t_tagzout = tr.translation_matrix(
                (trans.x * self.scale_x, trans.y * self.scale_y, trans.z * self.scale_z)
            )
            camzout_R_tagzout = tr.quaternion_matrix((rot.x, rot.y, rot.z, rot.w))
            camzout_T_tagzout = tr.concatenate_matrices(camzout_t_tagzout, camzout_R_tagzout)

            # What I think is going on:
            # We take the apriltag and rotate it in the apriltag coordinate
            # system (i.e. origin is at the apriltag).
            # Then we rotate the apriltag based on the camera coordinate system
            # (i.e. we apply the camera's rotation to the apriltag).
            # Then we translate the apriltag into its position in the camera
            # coordinate system
            # Finally, we rotate and translate the apriltag based on the robot's
            # coordinate system.
            # The value here is then the apriltag coordinates w.r.t. the robot's
            # baselink frame
            veh_T_tagxout = tr.concatenate_matrices(veh_T_camzout, camzout_T_tagzout, tagzout_T_tagxout)

            # Overwrite transformed value. They were the pose of the apriltag in
            # the camera frmae, now change them to be the pose of the apriltag
            # in the robot's baselink frame TODO: maybe we should use this!
            (trans.x, trans.y, trans.z) = tr.translation_from_matrix(veh_T_tagxout)
            (rot.x, rot.y, rot.z, rot.w) = tr.quaternion_from_matrix(veh_T_tagxout)

            new_tag_data.detections.append(detection)

        else:
            self._switch_led(id_info)

            tag_id = id_info["tag_id"]
            estimated_tag_frame_id = f"tag/{tag_id}"  # Where duckiebot thinks tag is
            tag_frame_id = f"at_fixed_{tag_id}"  # Ground truth
            transformed_frame = "transformed_" + tag_frame_id

            try:
                # We only want to update with an apriltag if we know its static
                # location
                estimate_loc = tag_frame_id in self.static_frames
                if estimate_loc:
                    # Get known location of apriltag
                    known_loc = self.static_frames[tag_frame_id]
                    x = known_loc.transform.translation.x
                    y = known_loc.transform.translation.y

                    # Get the transform from the robot's estimated location of
                    # the apriltag (frame tag/XX) to the robot's odometry
                    # position. Use the world coordinate system for these coordinates.
                    trans = self.tf_buffer.lookup_transform_full(
                        estimated_tag_frame_id,
                        rospy.Time(0),
                        "odometry",
                        rospy.Time(0),
                        "world",
                    )

                    # Broadcast this transform as transformed_at_fixed_XX. This
                    # is the transformation to apply to the fixed apriltag
                    # position (ground truth) to get the current estimate of
                    # the robot's position using the apriltag.
                    q_rot = [
                        trans.transform.rotation.x,
                        trans.transform.rotation.y,
                        trans.transform.rotation.z,
                        trans.transform.rotation.w,
                    ]
                    t = [
                        trans.transform.translation.x,
                        trans.transform.translation.y,
                        trans.transform.translation.z,
                    ]
                    self._tf_bcaster.sendTransform(
                        t,
                        q_rot,
                        rospy.Time.now(),
                        transformed_frame,
                        tag_frame_id,
                    )

                    # Get the transformation from the estimated apriltag
                    # position (tag/XX) to the world frame (origin), and apply
                    # it at the fixed frame of at_fixed_XX. This is the robot's
                    # position, estimated using apriltags.
                    at_trans = self.tf_buffer.lookup_transform_full(
                        "world",
                        rospy.Time(0),
                        transformed_frame,
                        rospy.Time(0),
                        tag_frame_id,
                    )

                    # Pack the previous transform into a pose
                    qx = at_trans.transform.rotation.x
                    qy = at_trans.transform.rotation.y
                    qz = at_trans.transform.rotation.z
                    qw = at_trans.transform.rotation.w
                    x = at_trans.transform.translation.x
                    y = at_trans.transform.translation.y
                    z = at_trans.transform.translation.z
                    updated_pose_at = Pose()
                    updated_pose_at.position.x = x
                    updated_pose_at.position.y = y
                    updated_pose_at.position.z = z
                    updated_pose_at.orientation.x = qx
                    updated_pose_at.orientation.y = qy
                    updated_pose_at.orientation.z = qz
                    updated_pose_at.orientation.w = qw

                    # Update the robot's guess of its current position to the
                    # estimate using apriltags
                    self.update_odom.publish(updated_pose_at)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

        new_tag_data.infos = tag_infos

        # Publish Message
        self.pub_postPros.publish(new_tag_data)


if __name__ == "__main__":
    rospy.init_node("apriltag_postprocessing_node", anonymous=False)
    node = AprilPostPros()
    rospy.spin()
