
#include "map_helper.h"

using namespace std;
using namespace MapLocalization;

int main(int argc, char **argv){
  ros::init(argc, argv, "map_locallization");
  
  ROS_INFO("\033[1;32m----> Global PCD Map Helper Started.\033[0m");
  GlobalPCDMapHelper map_helper;

  rosbag::Bag bag_out;
  if (map_helper.to_rosbag){
    bag_out.open(map_helper.str_HOME + map_helper.saved_PCD_directory + "pcd_map.bag", rosbag::bagmode::Write);
  }

  // these global messages just publish once.
  // global map
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = map_helper.world_coordinate_name;
  if(!map_helper.to_rosbag) {
    PublishPointCloud2(map_helper.global_corner_map_pub, *map_helper.global_corner_map, header);
    PublishPointCloud2(map_helper.global_surf_map_pub, *map_helper.global_surf_map, header);
    PublishPointCloud2(map_helper.global_map_pub, *map_helper.global_map, header);
    PublishPointCloud2(map_helper.global_gt_trajectory_pub, *map_helper.global_gt_trajectory, header);
    PublishPointCloud2(map_helper.global_gt_transforms_pub, *map_helper.global_gt_transformations, header);
  } else {
    PublishPointCloud2(map_helper.global_corner_map_pub, *map_helper.global_corner_map, header, map_helper.global_corner_map_topic, bag_out);
    PublishPointCloud2(map_helper.global_surf_map_pub, *map_helper.global_surf_map, header, map_helper.global_surf_map_topic, bag_out);
    PublishPointCloud2(map_helper.global_map_pub, *map_helper.global_map, header, map_helper.global_map_topic, bag_out);
    PublishPointCloud2(map_helper.global_gt_trajectory_pub, *map_helper.global_gt_trajectory, header, map_helper.global_gt_trajectory_topic, bag_out);
    PublishPointCloud2(map_helper.global_gt_transforms_pub, *map_helper.global_gt_transformations, header, map_helper.global_gt_transformations_topic, bag_out);
  }
  
  // simulate ros pcd message publi sher with 10 Hz
  ros::Rate pub_rate(10);
  const vector<string> &corner_splits = map_helper.global_corner_splits;
  const vector<string> &surf_splits = map_helper.global_surf_splits;
  const vector<string> &deskewed_pointclouds = map_helper.deskewed_pointclouds;
  
  int iter = 0;
  pcl::PointCloud<PointType>::Ptr corner_split_temp(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr surf_split_temp(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr deskewed_pointcloud_temp(new pcl::PointCloud<PointType>());

  while(iter < (int)deskewed_pointclouds.size() && ros::ok()) {
    header.stamp = ros::Time::now();
    // corner_splits and surf_splits, lost timestamp infomation of them in the current compare to lio_sam.
    if(iter < (int)corner_splits.size()) {
      LoadPCDFromLocal(corner_splits[iter].c_str(), *corner_split_temp);
      LoadPCDFromLocal(surf_splits[iter].c_str(), *surf_split_temp);
      header.frame_id = map_helper.world_coordinate_name;
      if(!map_helper.to_rosbag) {
        PublishPointCloud2(map_helper.global_corner_splits_pub, *corner_split_temp, header);
        PublishPointCloud2(map_helper.global_surf_splits_pub, *surf_split_temp, header);
      } else {
        PublishPointCloud2(map_helper.global_corner_splits_pub, *corner_split_temp, header, map_helper.global_corner_splits_topic, bag_out);
        PublishPointCloud2(map_helper.global_surf_splits_pub, *surf_split_temp, header, map_helper.global_surf_splits_topic, bag_out);
      }
    }
    // input lidar data for odometry map localization
    header.frame_id = map_helper.lidar_coordinate_name;
    LoadPCDFromLocal(deskewed_pointclouds[iter].c_str(), *deskewed_pointcloud_temp);
    if(!map_helper.to_rosbag) {
      PublishPointCloud2(map_helper.deskewed_pointcloud_pub, *deskewed_pointcloud_temp, header);
    } else {
      PublishPointCloud2(map_helper.deskewed_pointcloud_pub, *deskewed_pointcloud_temp, header, map_helper.deskewed_pointclouds_topic, bag_out);
    }
    iter++;
    pub_rate.sleep();
  }
  bag_out.close();
  std::cout << "Load pcd map done. \n";
  // ros::spin();
  return 0;
}