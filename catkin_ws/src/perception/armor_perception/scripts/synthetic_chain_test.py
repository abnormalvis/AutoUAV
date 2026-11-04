#!/usr/bin/env python3
"""End-to-end chain check against a synthetic camera image.

Publishes camera_info, odometry and a rendered image of the plate, then asserts
that /perception/target_pose lands on the plate's true world pose.

Why this exists: the highest-risk failure in the whole package is the
optical-frame convention. Get it wrong and the detector still finds the plate,
the debug image still looks perfect, and only the published world position is
wrong -- by a rotation, so it can even look plausible. This script renders the
plate through the *textbook* ROS convention (REP-103 body FLU, optical frame
+Z forward / +X right / +Y down) written out independently of the C++ code, so
agreement between the two is a real check and not a tautology.

Run via rostest:
    rostest armor_perception synthetic_chain.test
or by hand next to a running node:
    rosrun armor_perception synthetic_chain_test.py
Exit code 0 means the chain is consistent.
"""

import math
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool

# --- ground truth ---------------------------------------------------------
# Plate 2 m in front of the camera, normal facing back at it (yaw = pi).
PLATE_POSITION = np.array([3.0, 0.0, 1.2])
PLATE_YAW = math.pi

# Body pose held during the test: the camera sits 0.1 m ahead of base_link
# (matching ~cam_xyz), so the plate is 0.9 m away -- well inside the SITL
# camera's ~3 m detection range, and close enough that PnP depth error stays
# small. At 2 m the same chain measures ~2 cm instead of ~1 cm purely from
# corner rasterisation, so a tighter tolerance here would test the renderer
# rather than the code.
BODY_POSITION = np.array([2.0, 0.0, 1.2])
CAM_OFFSET_IN_BODY = np.array([0.1, 0.0, 0.0])

# Gazebo iris_vio intrinsics: 848x480, HFOV 1.5009831567 rad, no distortion.
WIDTH, HEIGHT = 848, 480
FX = FY = 455.4
CX, CY = 424.0, 240.0

# Plate geometry, matching uav_sitl.yaml and models/armor_plate.
LIGHTBAR_LENGTH = 0.056
LIGHTBAR_WIDTH = 0.018
SMALL_ARMOR_WIDTH = 0.135

TOLERANCE = 0.03  # m, position error budget
FRAMES = 40


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def plate_rotation(yaw):
    """R_world_armor: +X is the outward normal, +Z is up."""
    normal = rot_z(yaw) @ np.array([1.0, 0.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(up, normal)
    return np.column_stack([normal, right, up])


def quaternion_from_matrix(R):
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return (x, y, z, w)


SUPERSAMPLE = 4


def render(T_world_camera):
    """Draws the plate's two light bars as the camera would see them.

    Rendered at SUPERSAMPLE x resolution and box-filtered down. At 2 m the plate
    is only ~32 px wide, so drawing it directly with integer-pixel polygons puts
    a couple of pixels of edge error into the corners, which a planar PnP turns
    into centimetres of depth error -- enough to look like a chain bug when it is
    only rasterisation.
    """
    scale = SUPERSAMPLE
    K = np.array(
        [
            [FX * scale, 0.0, CX * scale],
            [0.0, FY * scale, CY * scale],
            [0.0, 0.0, 1.0],
        ]
    )
    dist = np.zeros(5)

    R_world_camera = T_world_camera[:3, :3]
    p_world_camera = T_world_camera[:3, 3]

    R_armor_world = plate_rotation(PLATE_YAW)
    R_camera_armor = R_world_camera.T @ R_armor_world
    p_camera_armor = R_world_camera.T @ (PLATE_POSITION - p_world_camera)

    rvec, _ = cv2.Rodrigues(R_camera_armor)
    tvec = p_camera_armor.reshape(3, 1)

    image = np.zeros((HEIGHT * scale, WIDTH * scale, 3), dtype=np.uint8)
    hy = LIGHTBAR_WIDTH / 2.0
    hz = LIGHTBAR_LENGTH / 2.0

    for y_center in (+SMALL_ARMOR_WIDTH / 2.0, -SMALL_ARMOR_WIDTH / 2.0):
        corners = np.array(
            [
                [0.0, y_center - hy, hz],
                [0.0, y_center + hy, hz],
                [0.0, y_center + hy, -hz],
                [0.0, y_center - hy, -hz],
            ],
            dtype=np.float64,
        )
        projected, _ = cv2.projectPoints(corners, rvec, tvec, K, dist)
        projected = projected.reshape(-1, 2)
        if not np.all(np.isfinite(projected)):
            continue
        cv2.fillPoly(image, [np.round(projected).astype(np.int32)], (255, 255, 255))

    return cv2.resize(image, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)


def main():
    rospy.init_node("synthetic_chain_test", anonymous=True)
    bridge = CvBridge()

    image_pub = rospy.Publisher("/camera/left/image_raw", Image, queue_size=2)
    info_pub = rospy.Publisher("/camera/left/camera_info", CameraInfo, queue_size=2)
    odom_pub = rospy.Publisher("/localization/odometry", Odometry, queue_size=10)
    valid_pub = rospy.Publisher("/vins_mavros_bridge/valid", Bool, queue_size=2, latch=True)

    received = []

    def on_target(msg):
        p = msg.pose.pose.position
        received.append(
            np.array([p.x, p.y, p.z])
        )

    rospy.Subscriber("/perception/target_pose", PoseWithCovarianceStamped, on_target)

    R_world_body = rot_z(0.0)
    p_world_camera = BODY_POSITION + R_world_body @ CAM_OFFSET_IN_BODY

    # Optical axes in the body frame, written out from REP-103:
    #   optical +Z (forward) = body +X
    #   optical +X (right)   = body -Y   (body +Y is left)
    #   optical +Y (down)    = body -Z
    R_body_optical = np.array(
        [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ]
    )

    T_world_camera = np.eye(4)
    T_world_camera[:3, :3] = R_world_body @ R_body_optical
    T_world_camera[:3, 3] = p_world_camera

    image = render(T_world_camera)
    if image.sum() == 0:
        rospy.logerr("FAIL: the synthetic plate projected outside the image")
        return 1

    info = CameraInfo()
    info.header.frame_id = "camera_link"
    info.width = WIDTH
    info.height = HEIGHT
    info.distortion_model = "plumb_bob"
    info.D = [0.0] * 5
    info.K = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
    info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.P = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]

    qx, qy, qz, qw = quaternion_from_matrix(R_world_body)

    rospy.sleep(0.5)  # let the node connect
    valid_pub.publish(Bool(data=True))

    rate = rospy.Rate(30.0)
    for _ in range(FRAMES):
        stamp = rospy.Time.now()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = BODY_POSITION[0]
        odom.pose.pose.position.y = BODY_POSITION[1]
        odom.pose.pose.position.z = BODY_POSITION[2]
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom_pub.publish(odom)

        info.header.stamp = stamp
        info_pub.publish(info)

        msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_link"
        image_pub.publish(msg)

        rate.sleep()

    rospy.sleep(0.3)

    if not received:
        rospy.logerr(
            "FAIL: no /perception/target_pose. Either nothing was detected or the "
            "tracker never reached TRACKING (min_detect_count=%d).",
            5,
        )
        return 1

    errors = [np.linalg.norm(p - PLATE_POSITION) for p in received]
    mean_err = float(np.mean(errors))
    max_err = float(np.max(errors))

    rospy.loginfo("poses: %d, mean position error %.4f m, max %.4f m",
                  len(received), mean_err, max_err)

    if mean_err > TOLERANCE:
        rospy.logerr(
            "FAIL: mean position error %.3f m > %.3f m. Most likely the optical-frame "
            "rotation or the camera extrinsic is wrong.",
            mean_err, TOLERANCE,
        )
        return 1

    rospy.loginfo("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
