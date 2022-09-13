#ifndef MAP_LOCALIZATION_COMMON_UTILITY_H
#define MAP_LOCALIZATION_COMMON_UTILITY_H

#include "types.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosbag/bag.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <string>

namespace MapLocalization {

class ParamServer {
public:
  ParamServer() {
    nh.param<float>("pcd_map_localization/localizingGlobalMapLeafSize", localizing_global_map_ds_leaf_size, 4.0);
    nh.param<float>("pcd_map_localization/localizingDeskewedLeafSize", localizing_deskewed_leaf_size, 2.0);
    nh.param<double>("pcd_map_localization/globalMapDsSearchRadius", global_map_ds_search_radius, 50);
    nh.param<float>("pcd_map_localization/ndtResolution", ndt_resolution, 20);
    nh.param<float>("pcd_map_localization/ndtMaxIterations", ndt_max_iterations, 100);
    nh.param<float>("pcd_map_localization/ndtRotationEpsilon", ndt_rotation_epsilon, 0.01);

    nh.param<std::string>("pcd_map_localization/globalCornerSplitsTopic", global_corner_splits_topic, "global_map/corner_splits");
    nh.param<std::string>("pcd_map_localization/globalSurfSplitsTopic", global_surf_splits_topic, "global_map/surf_splits");
    nh.param<std::string>("pcd_map_localization/globalCornerMapTopic", global_corner_map_topic, "global_map/corner");
    nh.param<std::string>("pcd_map_localization/globalSurfMapTopic", global_surf_map_topic, "global_map/surf");
    nh.param<std::string>("pcd_map_localization/globalMapTopic", global_map_topic, "global_map/global");
    nh.param<std::string>("pcd_map_localization/gtTrajectoryTopic", global_gt_trajectory_topic, "global_map/gt_trajectory");
    nh.param<std::string>("pcd_map_localization/gtTransformationsTopic", global_gt_transformations_topic, "global_map/gt_transformations");
    nh.param<std::string>("pcd_map_localization/deskewedPointcloudTopic", deskewed_pointclouds_topic, "input/deskewed_pointcloud");

    // load map info and flag
    nh.param<bool>("pcd_map_localization/toRosBag", to_rosbag, true);
    nh.param<std::string>("pcd_map_localization/savedPCDDirectory", saved_PCD_directory, "/Datasets/PCDMap/");
    nh.param<std::string>("pcd_map_localization/savedCornerMapDirectory", saved_corner_map_directory, "corner/");
    nh.param<std::string>("pcd_map_localization/savedSurfMapDirectory", saved_surf_map_directory, "surf/");
    nh.param<std::string>("pcd_map_localization/savedDeskewedPointcloudDirectory", saved_deskewed_pointclouds_directory, "input_data/");

    nh.param<std::string>("pcd_map_localization/worldCoordinateName", world_coordinate_name, "world");
    nh.param<std::string>("pcd_map_localization/robotCoordinateName", robot_coordinate_name, "robot");
    nh.param<std::string>("pcd_map_localization/lidarCoordinateName", lidar_coordinate_name, "lidar");
    nh.param<std::string>("pcd_map_localization/odometeryCoordinateName", odometery_coordinate_name, "odometery");

    usleep(100);
  }
  float localizing_global_map_ds_leaf_size;
  float localizing_deskewed_leaf_size;
  double global_map_ds_search_radius;
  float ndt_resolution;
  float ndt_max_iterations;
  float ndt_rotation_epsilon;

  ros::NodeHandle nh;
  std::string global_corner_splits_topic;
  std::string global_surf_splits_topic;
  std::string global_corner_map_topic;
  std::string global_surf_map_topic;
  std::string global_map_topic;
  std::string global_gt_trajectory_topic;
  std::string global_gt_transformations_topic;
  std::string deskewed_pointclouds_topic;

  // Saved pcd, eg : trajectory, corner map
  bool to_rosbag;
  std::string saved_PCD_directory;
  std::string saved_corner_map_directory;
  std::string saved_surf_map_directory;
  std::string saved_deskewed_pointclouds_directory;

  // ROS Frames name
  std::string world_coordinate_name;
  std::string robot_coordinate_name;
  std::string lidar_coordinate_name;
  std::string odometery_coordinate_name;
};

template<typename PointT>
inline void LoadPCDFromLocal(const char* filename, pcl::PointCloud<PointT> &out_pcd) {
  ROS_INFO_STREAM("Load " << filename << " ... ");
  if(pcl::io::loadPCDFile<PointT>(filename, out_pcd) == -1) {
    ROS_FATAL_STREAM("File " << filename << " can not read");
  }
}

template<typename PointT>
inline void PublishPointCloud2(const ros::Publisher pub, 
                               const pcl::PointCloud<PointT> &pcd, 
                               const std_msgs::Header &header) {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(pcd, msg);
  msg.header.stamp = header.stamp;
  msg.header.frame_id = header.frame_id;
  pub.publish(msg);
}
template<typename PointT>
inline void PublishPointCloud2(const ros::Publisher pub, 
                               const pcl::PointCloud<PointT> &pcd, 
                               const std_msgs::Header &header,
                               const std::string &topic,
                               rosbag::Bag &bag_out) {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(pcd, msg);
  msg.header.stamp = header.stamp;
  msg.header.frame_id = header.frame_id;
  pub.publish(msg);
  bag_out.write(topic, header.stamp, msg);
}

class TimeHelper {
public:
    /**
     * @brief Get current system timestamp in ms.
     * @return  current timestamp in ms.
     */
    static int64_t now_ms() {
      int64_t e1 = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      return e1;
    }

    /**
     * @brief Get current system timestamp in us.
     * @return  current timestamp in us.
     */
    static int64_t now_us() {
      int64_t t = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      return t;
    }
};
} // end namespace MapLocalization
#endif // end MAP_LOCALIZATION_COMMON_UTILITY_H