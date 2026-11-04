#!/usr/bin/env python3
"""Score /perception/target_pose against the ground-truth plate pose.

The SITL world places the plate at a fixed pose, so the estimate can be scored
directly. Two numbers matter and they fail for different reasons:

  position error  -- catches a wrong optical-frame rotation, a wrong extrinsic,
                     or a wrong plate dimension;
  yaw error       -- catches a PnP branch flip and, separately, tells you
                     whether the observation point is going to be usable at all.

Usage:
  rosrun armor_perception evaluate_pose.py --truth 3.0 0.0 1.2 --truth-yaw 3.14159265358979

  # from a bag, at the end:
  rosrun armor_perception evaluate_pose.py --truth 3.0 0.0 1.2 --truth-yaw 3.14159265358979 --duration 20
"""

import argparse
import math
import sys

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


def wrap(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_of(q):
    """Yaw of a quaternion, for a plate whose normal is its +X axis."""
    # Rotation matrix column 0 is the plate normal; its heading is atan2(R10, R00).
    r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    r10 = 2.0 * (q.x * q.y + q.w * q.z)
    return math.atan2(r10, r00)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--truth", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"),
        help="ground-truth plate position in the world frame",
    )
    parser.add_argument(
        "--truth-yaw", type=float, default=0.0,
        help="ground-truth heading of the plate normal, radians",
    )
    parser.add_argument(
        "--duration", type=float, default=0.0,
        help="stop after this many seconds (0 = run until interrupted)",
    )
    parser.add_argument("--topic", default="/perception/target_pose")
    parser.add_argument(
        "--tolerance", type=float, default=0.10,
        help="position error budget in metres; exceeding it fails the run",
    )
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    truth = args.truth
    samples = []

    def callback(msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        err = math.sqrt(
            (p.x - truth[0]) ** 2 + (p.y - truth[1]) ** 2 + (p.z - truth[2]) ** 2
        )
        samples.append((err, wrap(yaw_of(q) - args.truth_yaw)))

    rospy.init_node("evaluate_pose", anonymous=True)
    rospy.Subscriber(args.topic, PoseWithCovarianceStamped, callback)

    rospy.loginfo("waiting for %s (ground truth %.3f %.3f %.3f, yaw %.4f)",
                  args.topic, truth[0], truth[1], truth[2], args.truth_yaw)

    if args.duration > 0.0:
        rospy.sleep(args.duration)
    else:
        rospy.spin()

    if not samples:
        rospy.logerr("no poses received on %s", args.topic)
        return 1

    errors = [e for e, _ in samples]
    yaws = [abs(y) for _, y in samples]
    mean_err = sum(errors) / len(errors)
    max_err = max(errors)
    mean_yaw = sum(yaws) / len(yaws)

    rospy.loginfo("samples: %d", len(samples))
    rospy.loginfo("position error: mean %.3f m, max %.3f m", mean_err, max_err)
    rospy.loginfo("yaw error: mean %.1f deg, max %.1f deg",
                  math.degrees(mean_yaw), math.degrees(max(yaws)))

    if mean_err > args.tolerance:
        rospy.logerr("FAIL: mean position error %.3f m exceeds %.3f m",
                     mean_err, args.tolerance)
        return 1

    rospy.loginfo("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
