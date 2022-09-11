// for read data from local and publish message
#include "utility.h"
#include "types.h"

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>

#include <thread>

using namespace std;
using namespace MapLocalization;

class MapLocalizing : public ParamServer {
public:
  MapLocalizing() {
    allocate_memory();

    global_map_sub = nh.subscribe(global_map_topic, 1, &MapLocalizing::global_map_handle, this, ros::TransportHints().tcpNoDelay());
    global_gt_trajectory_sub = nh.subscribe(global_gt_trajectory_topic, 1, &MapLocalizing::global_gt_trajectory_handle, this, ros::TransportHints().tcpNoDelay());
    deskewed_sub = nh.subscribe(deskewed_pointclouds_topic, 2, &MapLocalizing::point_cloud_handle, this, ros::TransportHints().tcpNoDelay());

    global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_loc/global_map", 1);
    global_gt_trajectory_pub = nh.advertise<sensor_msgs::PointCloud2>("map_loc/global_gt_trajectory", 1);
    local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_loc/local_map", 10);
    odometry_trajectory_pub = nh.advertise<nav_msgs::Path>("map_loc/odometry_trajectory", 10);
  }
  void allocate_memory() {
    global_map_ptr.reset(new pcl::PointCloud<PointType>());
    global_corner_splits_ptr.reset(new pcl::PointCloud<PointType>());
    global_surf_splits_ptr.reset(new pcl::PointCloud<PointType>());
    global_gt_trajectory_ptr.reset(new pcl::PointCloud<PointType>());
    last_deskewed_ptr.reset(new pcl::PointCloud<PointType>());
    last_local_map_ptr.reset(new pcl::PointCloud<PointType>());
  }

  void global_map_handle(const sensor_msgs::PointCloud2::Ptr &msg_in) const{
    pcl::fromROSMsg(*msg_in, *global_map_ptr);
  }
  void global_gt_trajectory_handle(const sensor_msgs::PointCloud2::Ptr &msg_in) const{
    pcl::fromROSMsg(*msg_in, *global_gt_trajectory_ptr);
  }
  void point_cloud_handle(const sensor_msgs::PointCloud2::Ptr &msg_in) const{
    pcl::fromROSMsg(*msg_in, *last_deskewed_ptr);
  }
  /**
   * @brief Obviously it is a global constant pcd map in the current system.
   */
  void visualize_global_map_thread_func() {
    ros::Rate rate(0.1);
    while(ros::ok()) {
      rate.sleep();
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      header.frame_id = world_coordinate_name;
      // global map
      PublishPointCloud2(global_map_pub, *global_map_ptr, header);
      // global ground truth trajectory
      PublishPointCloud2(global_gt_trajectory_pub, *global_gt_trajectory_ptr, header);
    }
  }
  // Some pointer variables for converted message.
  pcl::PointCloud<PointType>::Ptr global_map_ptr;
  pcl::PointCloud<PointType>::Ptr global_corner_splits_ptr;
  pcl::PointCloud<PointType>::Ptr global_surf_splits_ptr;
  pcl::PointCloud<PointType>::Ptr global_gt_trajectory_ptr;
  pcl::PointCloud<PointType>::Ptr last_deskewed_ptr;
  pcl::PointCloud<PointType>::Ptr last_local_map_ptr;
  // Subscriber for input data
  ros::Subscriber global_map_sub;
  ros::Subscriber global_gt_trajectory_sub;
  ros::Subscriber deskewed_sub;
  // Publisher for some [intermediate] results
  ros::Publisher global_map_pub;
  ros::Publisher global_gt_trajectory_pub;
  ros::Publisher local_map_pub;
  ros::Publisher odometry_trajectory_pub;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_localizing");

  ROS_INFO("\033[1;32m----> Map Localizing Started.\033[0m");

  MapLocalizing map_localizing;
  std::thread visualize_global_map_thread(&MapLocalizing::visualize_global_map_thread_func, &map_localizing);

  ros::spin();
  visualize_global_map_thread.join();
  return 0;
}