#ifndef WAVEMAP_ROS_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_

// Error in eigen_conversions library
// #include <eigen_conversions/eigen_msg.h> 

// Manually include geometry_msgs types and convert
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

namespace wavemap::convert {
inline void rosMsgToPoint3D(const geometry_msgs::Vector3& msg, Point3D& point) {
  Eigen::Vector3d point_double;
  point_double << msg.x, msg.y, msg.z; // manual convert without eigen_conversions
  point = point_double.cast<FloatingPoint>();
}

inline void rosMsgToRotation3D(const geometry_msgs::Quaternion& msg,
                               Rotation3D& rotation) {
  Eigen::Quaterniond rotation_double;
  rotation_double = Eigen::Quaterniond{msg.w, msg.x, msg.y, msg.z}; // manual convert without eigen_conversions
  rotation = Rotation3D{rotation_double.cast<FloatingPoint>()};
}

inline void rosMsgToTransformation3D(const geometry_msgs::Transform& msg,
                                     Transformation3D& transform) {
  rosMsgToPoint3D(msg.translation, transform.getPosition());
  rosMsgToRotation3D(msg.rotation, transform.getRotation());
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_GEOMETRY_MSG_CONVERSIONS_H_
