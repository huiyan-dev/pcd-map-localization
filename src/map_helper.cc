#include "map_helper.h"

namespace MapLocalization {

using namespace std;

GlobalPCDMapHelper::GlobalPCDMapHelper() {

  allocate_memory();
  
  ROS_INFO_STREAM("savedPCDDirectory : " << saved_PCD_directory);
  ROS_INFO("Loading global pcd map ...");
  
  stringstream ss;
  str_HOME = std::getenv("HOME");
  if(str_HOME.empty()) {
    ROS_FATAL("HOME location wrong, please check your HOME directory.");
  }
  // Mixed global corner feature map
  ss.clear(), ss.str("");
  ss << str_HOME << saved_PCD_directory << "CornerMap.pcd";
  LoadPCDFromLocal<PointType>(ss.str().c_str(), *global_corner_map);
  // Mixed global surf feature map
  ss.clear(), ss.str("");
  ss << str_HOME << saved_PCD_directory << "SurfMap.pcd";
  LoadPCDFromLocal<PointType>(ss.str().c_str(), *global_surf_map);
  // Mixed global map for localization
  ss.clear(), ss.str("");
  ss << str_HOME << saved_PCD_directory << "GlobalMap.pcd";
  LoadPCDFromLocal<PointType>(ss.str().c_str(), *global_map);
  // Mixed Global trajectory as ground truth
  ss.clear(), ss.str("");
  ss << str_HOME << saved_PCD_directory << "trajectory.pcd";
  LoadPCDFromLocal<PointType>(ss.str().c_str(), *global_gt_trajectory);
  // Mixed transformation as ground truth
  ss.clear(), ss.str("");
  ss << str_HOME << saved_PCD_directory << "transformations.pcd";
  LoadPCDFromLocal<PointTypePose>(ss.str().c_str(), *global_gt_transformations);
  // Splits of global corner feature map
  ss.clear(), ss.str("");
  ss << str_HOME << saved_PCD_directory << saved_corner_map_directory;
  boost::filesystem::directory_iterator endDit;
  for(boost::filesystem::directory_iterator dit(ss.str()); dit != endDit; ++dit) {
    if(dit->path().extension() == ".pcd") {
      global_corner_splits.push_back(dit->path().string());
    }
  }
  // Splits of global surf feature map
  ss.clear(), ss.str("");
  ss << str_HOME << saved_PCD_directory << saved_surf_map_directory;
  for(boost::filesystem::directory_iterator dit(ss.str()); dit != endDit; ++dit) {
    if(dit->path().extension() == ".pcd") {
      global_surf_splits.push_back(dit->path().string());
    }
  }
  // Splits of global surf feature map
  ss.clear(), ss.str("");
  ss << str_HOME << saved_PCD_directory << saved_deskewed_pointclouds_directory;
  for(boost::filesystem::directory_iterator dit(ss.str()); dit != endDit; ++dit) {
    if(dit->path().extension() == ".pcd") {
      deskewed_pointclouds.push_back(dit->path().string());
    }
  }
  ROS_ASSERT(global_corner_splits.size() == global_surf_splits.size());
  sort(global_corner_splits.begin(), global_corner_splits.end());
  sort(global_surf_splits.begin(), global_surf_splits.end());
  sort(deskewed_pointclouds.begin(), deskewed_pointclouds.end());

  global_corner_splits_pub = nh.advertise<sensor_msgs::PointCloud2>(global_corner_splits_topic, 10);
  global_surf_splits_pub = nh.advertise<sensor_msgs::PointCloud2>(global_surf_splits_topic, 10);
  global_corner_map_pub = nh.advertise<sensor_msgs::PointCloud2>(global_corner_map_topic, 1);
  global_surf_map_pub = nh.advertise<sensor_msgs::PointCloud2>(global_surf_map_topic, 1);
  global_map_pub = nh.advertise<sensor_msgs::PointCloud2>(global_map_topic, 1);
  global_gt_trajectory_pub = nh.advertise<sensor_msgs::PointCloud2>(global_gt_trajectory_topic, 1);
  global_gt_transforms_pub = nh.advertise<sensor_msgs::PointCloud2>(global_gt_transformations_topic, 1);
  deskewed_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(deskewed_pointclouds_topic, 10);
}
void GlobalPCDMapHelper::allocate_memory() {
  global_surf_map.reset(new pcl::PointCloud<PointType>());
  global_corner_map.reset(new pcl::PointCloud<PointType>());
  global_map.reset(new pcl::PointCloud<PointType>());
  global_gt_trajectory.reset(new pcl::PointCloud<PointType>());
  global_gt_transformations.reset(new pcl::PointCloud<PointTypePose>());
}

} // end namespace MapLocalization