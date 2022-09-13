#ifndef MAP_LOCALIZATION_COMMON_TYPES_H
#define MAP_LOCALIZATION_COMMON_TYPES_H

#include <pcl/point_types.h>

// Lidar Type
enum class SensorLidarType {
  VELODYNE
};
// Camera Type
enum class CameraLidarType {
  PINHOLE,
  FISHEYE
};
// IMU Type
enum class CameraIMUType {
  SIX_AXIS,
  NINE_AXIS
};
// for pcl 6D Pose Data
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // adding a XYZ + padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;                    // SSE padding for memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;
typedef pcl::PointXYZI PointType;

#endif // end MAP_LOCALIZATION_COMMON_TYPES_H
