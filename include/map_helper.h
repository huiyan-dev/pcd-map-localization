#ifndef MAP_LOCALIZATION_MAP_HELPER_H
#define MAP_LOCALIZATION_MAP_HELPER_H

// for read data from local and publish message
#include "common/utility.h"
#include "common/types.h"

// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// for read/save pcd file
#include <pcl/io/pcd_io.h>

// Boost
#include <boost/filesystem.hpp>

#include <string>
namespace MapLocalization {

class GlobalPCDMapHelper : public ParamServer {
public:
  GlobalPCDMapHelper();
  ~GlobalPCDMapHelper() = default;

  void allocate_memory();

  std::string str_HOME;
  std::vector<std::string> global_corner_splits;
  std::vector<std::string> global_surf_splits;
  std::vector<std::string> deskewed_pointclouds;
  pcl::PointCloud<PointType>::Ptr global_corner_map;
  pcl::PointCloud<PointType>::Ptr global_surf_map;
  pcl::PointCloud<PointType>::Ptr global_map;
  pcl::PointCloud<PointType>::Ptr global_gt_trajectory;
  pcl::PointCloud<PointTypePose>::Ptr global_gt_transformations;

  ros::Publisher global_corner_splits_pub;
  ros::Publisher global_surf_splits_pub;
  ros::Publisher global_corner_map_pub;
  ros::Publisher global_surf_map_pub;
  ros::Publisher global_map_pub;
  ros::Publisher global_gt_trajectory_pub;
  ros::Publisher global_gt_transforms_pub;
  ros::Publisher deskewed_pointcloud_pub;
};

} // end namespace MapLocalization
#endif // end MAP_LOCALIZATION_MAP_HELPER_H
