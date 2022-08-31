#ifndef MAP_LOCALIZATION_UTILITY_H
#define MAP_LOCALIZATION_UTILITY_H

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
    nh_.param<std::string>("pcd_map_localization/globalCornerSplitsTopic", global_corner_splits_topic, "global_map/corner_splits");
    nh_.param<std::string>("pcd_map_localization/globalSurfSplitsTopic", global_surf_splits_topic, "global_map/surf_splits");
    nh_.param<std::string>("pcd_map_localization/globalCornerMapTopic", global_corner_map_topic, "global_map/corner");
    nh_.param<std::string>("pcd_map_localization/globalSurfMapTopic", global_surf_map_topic, "global_map/surf");
    nh_.param<std::string>("pcd_map_localization/globalMapTopic", global_map_topic, "global_map/global");
    nh_.param<std::string>("pcd_map_localization/gtTrajectoryTopic", global_gt_trajectory_topic, "global_map/gt_trajectory");
    nh_.param<std::string>("pcd_map_localization/gtTransformationsTopic", global_gt_transformations_topic, "global_map/gt_transformations");

    // load map info and flag
    nh_.param<bool>("pcd_map_localization/toRosBag", to_rosbag, true);
    nh_.param<std::string>("pcd_map_localization/savedPCDDirectory", saved_PCD_directory, "/Datasets/PCDMap/");
    nh_.param<std::string>("pcd_map_localization/savedCornerMapDirectory", saved_corner_map_directory, "corner/");
    nh_.param<std::string>("pcd_map_localization/savedSurfMapDirectory", saved_surf_map_directory, "surf/");

    nh_.param<std::string>("pcd_map_localization/worldCoordinateName", world_coordinate_name, "world");
    nh_.param<std::string>("pcd_map_localization/robotCoordinateName", robot_coordinate_name, "robot");
    nh_.param<std::string>("pcd_map_localization/lidarCoordinateName", lidar_coordinate_name, "lidar");
    
    usleep(100);
  }

  ros::NodeHandle nh_;
  std::string global_corner_splits_topic;
  std::string global_surf_splits_topic;
  std::string global_corner_map_topic;
  std::string global_surf_map_topic;
  std::string global_map_topic;
  std::string global_gt_trajectory_topic;
  std::string global_gt_transformations_topic;

  // Saved pcd, eg : trajectory, corner map
  bool to_rosbag;
  std::string saved_PCD_directory;
  std::string saved_corner_map_directory;
  std::string saved_surf_map_directory;

  // ROS Frames name
  std::string world_coordinate_name;
  std::string robot_coordinate_name;
  std::string lidar_coordinate_name;
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
} // end namespace MapLocalization
#endif // end MAP_LOCALIZATION_UTILITY_H