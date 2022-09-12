// for read data from local and publish message
#include "utility.h"
#include "types.h"

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>

#include <thread>

using namespace std;
using namespace MapLocalization;

class MapLocalizing : public ParamServer {
public:
  MapLocalizing() {
    allocate_memory();

    is_initialized = false;
    // Publish TF
    tf::TransformBroadcaster br;


    global_map_sub = nh.subscribe(global_map_topic, 1, &MapLocalizing::global_map_handle, this, ros::TransportHints().tcpNoDelay());
    global_gt_trajectory_sub = nh.subscribe(global_gt_trajectory_topic, 1, &MapLocalizing::global_gt_trajectory_handle, this, ros::TransportHints().tcpNoDelay());
    global_gt_transformations_sub = nh.subscribe(global_gt_transformations_topic, 1, &MapLocalizing::global_gt_transformations_handle, this, ros::TransportHints().tcpNoDelay());
    deskewed_sub = nh.subscribe(deskewed_pointclouds_topic, 2, &MapLocalizing::point_cloud_handle, this, ros::TransportHints().tcpNoDelay());

    global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_loc/global_map", 1);
    global_gt_trajectory_pub = nh.advertise<sensor_msgs::PointCloud2>("map_loc/global_gt_trajectory", 1);
    local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_loc/local_map", 10);
    odometry_trajectory_pub = nh.advertise<nav_msgs::Path>("map_loc/odometry_trajectory", 10);
    pub_icp_odometry = nh.advertise<nav_msgs::Odometry>("map_loc/odomtery", 10);
  }
  void allocate_memory() {
    global_map_ptr.reset(new pcl::PointCloud<PointType>());
    global_corner_splits_ptr.reset(new pcl::PointCloud<PointType>());
    global_surf_splits_ptr.reset(new pcl::PointCloud<PointType>());
    global_gt_trajectory_ptr.reset(new pcl::PointCloud<PointType>());
    global_gt_transformations_ptr.reset(new pcl::PointCloud<PointTypePose>());
    last_deskewed_ptr.reset(new pcl::PointCloud<PointType>());
    last_local_map_ptr.reset(new pcl::PointCloud<PointType>());

  }
  void reset_system() {
    is_initialized = false;
  }
  void global_map_handle(const sensor_msgs::PointCloud2::Ptr &msg_in) const{
    pcl::fromROSMsg(*msg_in, *global_map_ptr);
  }
    void  global_gt_transformations_handle(const sensor_msgs::PointCloud2::Ptr &msg_in) {
      pcl::fromROSMsg(*msg_in, *global_gt_transformations_ptr);
      // This initial pose from a handed add by ground gruth.
      transformTobeMapped[3] = (*global_gt_transformations_ptr).points[0].x;
      transformTobeMapped[4] = (*global_gt_transformations_ptr).points[0].y;
      transformTobeMapped[5] = (*global_gt_transformations_ptr).points[0].z;
      transformTobeMapped[0] = (*global_gt_transformations_ptr).points[0].roll;
      transformTobeMapped[1] = (*global_gt_transformations_ptr).points[0].pitch;
      transformTobeMapped[2] = (*global_gt_transformations_ptr).points[0].yaw;
    }
  void global_gt_trajectory_handle(const sensor_msgs::PointCloud2::Ptr &msg_in) const{
    pcl::fromROSMsg(*msg_in, *global_gt_trajectory_ptr);
  }
  void point_cloud_handle(const sensor_msgs::PointCloud2::Ptr &msg_in) {
    pcl::fromROSMsg(*msg_in, *last_deskewed_ptr);

    if(!is_initialized) {
      is_initialized = true;
      return;
    }
    last_laser_header = (*msg_in).header;
    std::lock_guard<std::mutex> lock(localizing_mutex);
    // get local map
    update_local_map();
    localizing();
    // publish some results
    PublishPointCloud2(local_map_pub, *last_local_map_ptr, last_laser_header);
    // publish odometry from icp localizing.
    publish_odometry();
  }
    void publish_odometry()
    {
      // Publish odometry for ROS (global)
      // 发送当前帧的位姿
      nav_msgs::Odometry icp_odometry;
      icp_odometry.header.stamp = last_laser_header.stamp;
      icp_odometry.header.frame_id = world_coordinate_name;
      icp_odometry.child_frame_id = odometery_coordinate_name;
      icp_odometry.pose.pose.position.x = transformTobeMapped[3];
      icp_odometry.pose.pose.position.y = transformTobeMapped[4];
      icp_odometry.pose.pose.position.z = transformTobeMapped[5];
      icp_odometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
      pub_icp_odometry.publish(icp_odometry);

      // Publish TF
      // lidar to odometry
      static tf::TransformBroadcaster br;
      tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                    tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
      tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, last_laser_header.stamp, odometery_coordinate_name, lidar_coordinate_name);
      br.sendTransform(trans_odom_to_lidar);
      // odometry to world (odometry coordinate is same as world in the current version)
      tf::Transform t_odometry_to_world = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
      tf::StampedTransform trans_odometry_to_world = tf::StampedTransform(t_odometry_to_world, last_laser_header.stamp, world_coordinate_name, odometery_coordinate_name);
      br.sendTransform(trans_odometry_to_world);
    }
  void update_local_map() {

  }
  void localizing() {

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
  bool is_initialized;
  std::mutex localizing_mutex;
  float transformTobeMapped[6];
  std_msgs::Header last_laser_header;


  // Some pointer variables for converted message.
  pcl::PointCloud<PointType>::Ptr global_map_ptr;
  pcl::PointCloud<PointType>::Ptr global_corner_splits_ptr;
  pcl::PointCloud<PointType>::Ptr global_surf_splits_ptr;
  pcl::PointCloud<PointType>::Ptr global_gt_trajectory_ptr; // PointType
  pcl::PointCloud<PointTypePose>::Ptr global_gt_transformations_ptr;  // PointTypePose
  pcl::PointCloud<PointType>::Ptr last_deskewed_ptr;
  pcl::PointCloud<PointType>::Ptr last_local_map_ptr;
  // Subscriber for input data
  ros::Subscriber global_map_sub;
  ros::Subscriber global_gt_trajectory_sub;
  ros::Subscriber global_gt_transformations_sub;
  ros::Subscriber deskewed_sub;
  // Publisher for some [intermediate] results
  ros::Publisher global_map_pub;
  ros::Publisher global_gt_trajectory_pub;
  ros::Publisher local_map_pub;
  ros::Publisher odometry_trajectory_pub;
  ros::Publisher pub_icp_odometry;
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