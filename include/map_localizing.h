#ifndef MAP_LOCALIZATION_MAP_LOCALIZING_H
#define MAP_LOCALIZATION_MAP_LOCALIZING_H

// for read data from local and publish message
#include "common/utility.h"
#include "common/types.h"

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include <thread>

namespace MapLocalization {

class MapLocalizing : public ParamServer {
public:
  MapLocalizing();
  ~MapLocalizing() = default;
  void allocate_memory();
  void reset_system();
  void global_map_handle(const sensor_msgs::PointCloud2::Ptr &msg_in);
  void global_gt_transformations_handle(const sensor_msgs::PointCloud2::Ptr &msg_in);
  void global_gt_trajectory_handle(const sensor_msgs::PointCloud2::Ptr &msg_in) const;
  void point_cloud_handle(const sensor_msgs::PointCloud2::Ptr &msg_in);
  void down_sample_deskewed();
  void publish_odometry();
  PointType get_last_point_type();
  PointTypePose get_last_point_type_pose();
  void update_local_map();
  void localizing();
  void visualize_global_map_thread_func();

  bool is_system_initialized;
  bool is_initial_pose_initialized;
  bool init_ndt;
  pcl::NormalDistributionsTransform<PointType, PointType> ndt;
  Eigen::Matrix4f delta_pose, last_pose;
  std::mutex localizing_mutex;
  float transformTobeMapped[6];
  float lastTransformTobeMapped[6];
  std_msgs::Header last_laser_header;
  nav_msgs::Path global_trajectories;
  // for global map search
  pcl::KdTreeFLANN<PointType>::Ptr kdtree_global_map_ds;
  // for global map down sample
  pcl::VoxelGrid<PointType> down_sample_filter_global_map;
  pcl::VoxelGrid<PointType> down_sample_filter_deskewed;
  std::size_t  global_map_ds_num;
  // Some pointer variables for converted message.
  pcl::PointCloud<PointType>::Ptr global_map_ptr;
  pcl::PointCloud<PointType>::Ptr global_map_ds_ptr;
  pcl::PointCloud<PointType>::Ptr global_corner_splits_ptr;
  pcl::PointCloud<PointType>::Ptr global_surf_splits_ptr;
  pcl::PointCloud<PointType>::Ptr global_gt_trajectory_ptr; // PointType
  pcl::PointCloud<PointTypePose>::Ptr global_gt_transformations_ptr;  // PointTypePose
  pcl::PointCloud<PointType>::Ptr last_deskewed_ptr;
  pcl::PointCloud<PointType>::Ptr last_deskewed_ds_ptr;
  pcl::PointCloud<PointType>::Ptr last_local_map_ptr;
  // Subscriber for input data
  ros::Subscriber global_map_sub;
  ros::Subscriber global_gt_trajectory_sub;
  ros::Subscriber global_gt_transformations_sub;
  ros::Subscriber deskewed_sub;
  // Publisher for some [intermediate] results
  ros::Publisher global_map_pub;
  ros::Publisher global_map_ds_pub;
  ros::Publisher global_gt_trajectory_pub;
  ros::Publisher deskewed_ds_pub;
  ros::Publisher local_map_pub;
  ros::Publisher odometry_trajectory_pub;
  ros::Publisher pub_icp_odometry;
};

} // end namespace MapLocalization
#endif // end MAP_LOCALIZATION_MAP_LOCALIZING_H