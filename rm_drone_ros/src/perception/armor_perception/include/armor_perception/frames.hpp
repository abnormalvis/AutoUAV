#ifndef ARMOR_PERCEPTION__FRAMES_HPP
#define ARMOR_PERCEPTION__FRAMES_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace armor_perception
{

// Rotation taking a vector expressed in the *body* frame into the *optical*
// frame.
//
// Body (base_link, REP-103): +X forward, +Y left, +Z up.
// Optical (OpenCV / ROS camera_optical_frame): +Z forward, +X right, +Y down.
//
//     body +X (forward) -> optical +Z
//     body +Y (left)    -> optical -X
//     body +Z (up)      -> optical -Y
//
// Gazebo Classic's camera sensor uses the body convention for its link, so the
// same matrix converts an iris_vio camera_link into what cv::solvePnP expects.
//
// This is the step that silently rotates every pose if it is wrong: detections
// still look perfect in the debug image while the world-frame target position
// is somewhere else entirely. buildTBaseOptical() is therefore tested directly,
// and the node's reprojection self-check exercises the whole chain.
inline Eigen::Matrix3d opticalFromBody()
{
  return (Eigen::Matrix3d() << 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0).finished();
}

// T_base_optical from the camera's mounting pose in the body frame.
//
// `p_camera_in_body` is the camera origin in base_link.
// `R_body_camera` maps camera-link vectors into body coordinates (its columns
// are the camera link's axes expressed in the body frame); pass the identity
// for a camera whose link already uses the body convention -- the usual case,
// including iris_vio's camera_link.
inline Eigen::Isometry3d buildTBaseOptical(
  const Eigen::Vector3d & p_camera_in_body, const Eigen::Matrix3d & R_body_camera)
{
  // v_optical = R_ob * v_cameralink and v_base = R_body_cameralink * v_cameralink,
  // so v_base = (R_body_cameralink * R_ob^T) * v_optical.
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = R_body_camera * opticalFromBody().transpose();
  T.translation() = p_camera_in_body;
  return T;
}

}  // namespace armor_perception

#endif  // ARMOR_PERCEPTION__FRAMES_HPP
