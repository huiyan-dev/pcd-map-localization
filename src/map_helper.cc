// for read data from local and publish message
#include "utility.h"
#include "types.h"

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

using namespace std;
using namespace MapLocalization;

class GlobalPCDMapHelper : public ParamServer {
public:
  GlobalPCDMapHelper() {

    allocateMemory();
    
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
    ROS_ASSERT(global_corner_splits.size() == global_surf_splits.size());
    sort(global_corner_splits.begin(), global_corner_splits.end());
    sort(global_surf_splits.begin(), global_surf_splits.end());

    global_corner_splits_pub = nh_.advertise<sensor_msgs::PointCloud2>(global_corner_splits_topic, 10);
    global_surf_splits_pub = nh_.advertise<sensor_msgs::PointCloud2>(global_surf_splits_topic, 10);
    global_corner_map_pub = nh_.advertise<sensor_msgs::PointCloud2>(global_corner_map_topic, 1);
    global_surf_map_pub = nh_.advertise<sensor_msgs::PointCloud2>(global_surf_map_topic, 1);
    global_map_pub = nh_.advertise<sensor_msgs::PointCloud2>(global_map_topic, 1);
    global_gt_trajectory_pub = nh_.advertise<sensor_msgs::PointCloud2>(global_gt_trajectory_topic, 1);
    global_gt_transforms_pub = nh_.advertise<sensor_msgs::PointCloud2>(global_gt_transformations_topic, 1);

  }
  ~GlobalPCDMapHelper() {
    
  }
  void allocateMemory() {
    global_surf_map.reset(new pcl::PointCloud<PointType>());
    global_corner_map.reset(new pcl::PointCloud<PointType>());
    global_map.reset(new pcl::PointCloud<PointType>());
    global_gt_trajectory.reset(new pcl::PointCloud<PointType>());
    global_gt_transformations.reset(new pcl::PointCloud<PointTypePose>());
  }
  string str_HOME;
  vector<string> global_corner_splits;
  vector<string> global_surf_splits;
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
};
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
  
  // simulate ros pcd message publisher with 10 Hz
  ros::Rate pub_rate(10);
  const vector<string> &corner_splits = map_helper.global_corner_splits;
  const vector<string> &surf_splits = map_helper.global_surf_splits;
  int iter = 0;
  pcl::PointCloud<PointType>::Ptr corner_split_temp(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr surf_split_temp(new pcl::PointCloud<PointType>());

  while(iter < (int)corner_splits.size() && ros::ok()) {
    LoadPCDFromLocal(corner_splits[iter].c_str(), *corner_split_temp);
    LoadPCDFromLocal(surf_splits[iter].c_str(), *surf_split_temp);
    
    header.stamp = ros::Time::now();
    if(!map_helper.to_rosbag) {
      PublishPointCloud2(map_helper.global_corner_splits_pub, *corner_split_temp, header);
      PublishPointCloud2(map_helper.global_surf_splits_pub, *surf_split_temp, header);
    } else {
      PublishPointCloud2(map_helper.global_corner_splits_pub, *corner_split_temp, header, map_helper.global_corner_splits_topic, bag_out);
      PublishPointCloud2(map_helper.global_surf_splits_pub, *surf_split_temp, header, map_helper.global_surf_splits_topic, bag_out);
    }
    
    iter++;
    pub_rate.sleep();
  }
  bag_out.close();
  std::cout << "Load pcd map done. \n";
  // ros::spin();
  return 0;
}