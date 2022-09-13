#include "map_localizing.h"

namespace MapLocalization {
  
using namespace std;

MapLocalizing::MapLocalizing() {
  allocate_memory();

  is_system_initialized = false;
  is_initial_pose_initialized = false;
  init_ndt = true;
  ndt.setTransformationEpsilon (ndt_rotation_epsilon);
  ndt.setStepSize (0.05);
  ndt.setResolution (ndt_resolution);
  ndt.setMaximumIterations (ndt_max_iterations);
  delta_pose.setIdentity();
  last_pose.setIdentity();
  global_map_ds_num = 0;
  // global map down sample leaf size
  down_sample_filter_global_map.setLeafSize(localizing_global_map_ds_leaf_size, localizing_global_map_ds_leaf_size, localizing_global_map_ds_leaf_size);
  down_sample_filter_deskewed.setLeafSize(localizing_deskewed_leaf_size, localizing_deskewed_leaf_size, localizing_deskewed_leaf_size);

  global_map_sub = nh.subscribe(global_map_topic, 1, &MapLocalizing::global_map_handle, this, ros::TransportHints().tcpNoDelay());
  global_gt_trajectory_sub = nh.subscribe(global_gt_trajectory_topic, 1, &MapLocalizing::global_gt_trajectory_handle, this, ros::TransportHints().tcpNoDelay());
  global_gt_transformations_sub = nh.subscribe(global_gt_transformations_topic, 1, &MapLocalizing::global_gt_transformations_handle, this, ros::TransportHints().tcpNoDelay());
  deskewed_sub = nh.subscribe(deskewed_pointclouds_topic, 2, &MapLocalizing::point_cloud_handle, this, ros::TransportHints().tcpNoDelay());

  global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_loc/global_map", 1);
  global_map_ds_pub = nh.advertise<sensor_msgs::PointCloud2>("map_loc/global_map_ds", 1);
  global_gt_trajectory_pub = nh.advertise<sensor_msgs::PointCloud2>("map_loc/global_gt_trajectory", 1);
  deskewed_ds_pub = nh.advertise<sensor_msgs::PointCloud2>("map_loc/deskewed_ds", 2);
  local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_loc/local_map", 10);
  odometry_trajectory_pub = nh.advertise<nav_msgs::Path>("map_loc/odometry_trajectory", 10);
  pub_icp_odometry = nh.advertise<nav_msgs::Odometry>("map_loc/odomtery", 10);

}

void MapLocalizing::allocate_memory() {
  global_map_ptr.reset(new pcl::PointCloud<PointType>());
  global_map_ds_ptr.reset(new pcl::PointCloud<PointType>());
  global_corner_splits_ptr.reset(new pcl::PointCloud<PointType>());
  global_surf_splits_ptr.reset(new pcl::PointCloud<PointType>());
  global_gt_trajectory_ptr.reset(new pcl::PointCloud<PointType>());
  global_gt_transformations_ptr.reset(new pcl::PointCloud<PointTypePose>());
  last_deskewed_ptr.reset(new pcl::PointCloud<PointType>());
  last_deskewed_ds_ptr.reset(new pcl::PointCloud<PointType>());
  last_local_map_ptr.reset(new pcl::PointCloud<PointType>());

  kdtree_global_map_ds.reset(new pcl::KdTreeFLANN<PointType>());
}
void MapLocalizing::reset_system() {
  is_initial_pose_initialized = false;
  is_system_initialized = false;
}
void MapLocalizing::global_map_handle(const sensor_msgs::PointCloud2::Ptr &msg_in) {
  pcl::fromROSMsg(*msg_in, *global_map_ptr);
  global_map_ds_ptr->clear();
  down_sample_filter_global_map.setInputCloud(global_map_ptr);
  down_sample_filter_global_map.filter(*global_map_ds_ptr);
  global_map_ds_num = global_map_ds_ptr->size();
  ROS_INFO_STREAM("global_map_ds_num : " << global_map_ds_num << '\n');
  ndt.setInputTarget(global_map_ds_ptr);
}
void MapLocalizing::global_gt_transformations_handle(const sensor_msgs::PointCloud2::Ptr &msg_in) {
  pcl::fromROSMsg(*msg_in, *global_gt_transformations_ptr);
  // This initial pose from a handed add by ground gruth.
  transformTobeMapped[3] = (*global_gt_transformations_ptr).points[0].x;
  transformTobeMapped[4] = (*global_gt_transformations_ptr).points[0].y;
  transformTobeMapped[5] = (*global_gt_transformations_ptr).points[0].z;
  transformTobeMapped[0] = (*global_gt_transformations_ptr).points[0].roll;
  transformTobeMapped[1] = (*global_gt_transformations_ptr).points[0].pitch;
  transformTobeMapped[2] = (*global_gt_transformations_ptr).points[0].yaw;
//    for(auto &i : transformTobeMapped) {
//      i = 0;
//    }
  ROS_INFO_STREAM("Init pose : " << ", x : " << transformTobeMapped[3] << ", y : " << transformTobeMapped[4] << ", z : "
                                  << transformTobeMapped[5] << ", roll : " << transformTobeMapped[0] << ", pitch : "
                                  << transformTobeMapped[1] << ", yaw : " << transformTobeMapped[2]);

  // odometry to world (odometry coordinate is same as world in the current version)
  publish_odometry();

  is_initial_pose_initialized = true;
}
void MapLocalizing::global_gt_trajectory_handle(const sensor_msgs::PointCloud2::Ptr &msg_in) const{
  pcl::fromROSMsg(*msg_in, *global_gt_trajectory_ptr);
}
void MapLocalizing::point_cloud_handle(const sensor_msgs::PointCloud2::Ptr &msg_in) {
  // A initial pose at system start time
  if (!is_initial_pose_initialized) {
    ROS_INFO("is_initial_pose_initialized == false");
    return;
  }
  pcl::fromROSMsg(*msg_in, *last_deskewed_ptr);
  last_laser_header = (*msg_in).header;
  // publish some results
  last_laser_header.frame_id = world_coordinate_name;
  PublishPointCloud2(local_map_pub, *last_local_map_ptr, last_laser_header);
  last_laser_header.frame_id = lidar_coordinate_name;
  PublishPointCloud2(deskewed_ds_pub, *last_deskewed_ds_ptr, last_laser_header);
  if(!is_system_initialized) {
    publish_odometry();
    is_system_initialized = true;
    return;
  }
//    std::lock_guard<std::mutex> lock(localizing_mutex);
  auto time_start = TimeHelper::now_ms();
  down_sample_deskewed();
  ROS_INFO_STREAM("down_sample_deskewed cost : " << (TimeHelper::now_ms() - time_start));
  time_start = TimeHelper::now_ms();
  ROS_INFO_STREAM("Input deskewed data down sample size : " << last_deskewed_ds_ptr->size());
  // get global local map
//    update_local_map();
//    ROS_INFO_STREAM("update_local_map cost : " << (TimeHelper::now_ms() - time_start));
//    time_start = TimeHelper::now_ms();
  // icp between global local map and lidar scan
  localizing();
  ROS_INFO_STREAM("localizing cost : " << (TimeHelper::now_ms() - time_start) << endl);
  // publish odometry from icp localizing.
  publish_odometry();
}
void MapLocalizing::down_sample_deskewed() {
  last_deskewed_ds_ptr->clear();
  down_sample_filter_deskewed.setInputCloud(last_deskewed_ptr);
  down_sample_filter_deskewed.filter(*last_deskewed_ds_ptr);
}
void MapLocalizing::publish_odometry()
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
  icp_odometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0],
                                                                                transformTobeMapped[1],
                                                                                transformTobeMapped[2]);
  pub_icp_odometry.publish(icp_odometry);


  geometry_msgs::PoseStamped traj_pose;
  traj_pose.header.stamp = last_laser_header.stamp;
  traj_pose.header.frame_id = odometery_coordinate_name;
  tf::Quaternion quat = tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1],
                                                                transformTobeMapped[2]);
  traj_pose.pose.orientation.x = quat.x();
  traj_pose.pose.orientation.y = quat.y();
  traj_pose.pose.orientation.y = quat.y();
  traj_pose.pose.orientation.z = quat.z();
  traj_pose.pose.position.x = transformTobeMapped[3];
  traj_pose.pose.position.y = transformTobeMapped[4];
  traj_pose.pose.position.z = transformTobeMapped[5];
  global_trajectories.poses.push_back(traj_pose);
  odometry_trajectory_pub.publish(global_trajectories);
  // Publish TF
  static tf::TransformBroadcaster br;
  // odometry to world
  tf::Transform t_world_odometry =
//        tf::Transform(tf::createQuaternionFromRPY(0,0,0),tf::Vector3(0,0,0));
    tf::Transform(
      tf::createQuaternionFromRPY((*global_gt_transformations_ptr).points[0].roll,
                                  (*global_gt_transformations_ptr).points[0].pitch,
                                  (*global_gt_transformations_ptr).points[0].yaw),
      tf::Vector3((*global_gt_transformations_ptr).points[0].x, (*global_gt_transformations_ptr).points[0].y,
                  (*global_gt_transformations_ptr).points[0].z));
  tf::StampedTransform trans_odometry_to_world = tf::StampedTransform(t_world_odometry, last_laser_header.stamp,
                                                                      world_coordinate_name,
                                                                      odometery_coordinate_name);
  br.sendTransform(trans_odometry_to_world);
  // lidar to world
  tf::Transform t_world_lidar =
//        tf::Transform(tf::createQuaternionFromRPY(0,0,0),tf::Vector3(0,0,0));

//        tf::Transform(
//            tf::createQuaternionFromRPY(lastTransformTobeMapped[0], lastTransformTobeMapped[1], lastTransformTobeMapped[2]),
//            tf::Vector3(lastTransformTobeMapped[3], lastTransformTobeMapped[4], lastTransformTobeMapped[5])) *
      tf::Transform(
          quat,
      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
  for(int i  = 0; i < 6; ++i) lastTransformTobeMapped[i] = transformTobeMapped[i];
  tf::StampedTransform trans_lidar_to_odometry = tf::StampedTransform(t_world_lidar, last_laser_header.stamp,
                                                                  world_coordinate_name, lidar_coordinate_name);
  br.sendTransform(trans_lidar_to_odometry);

}
PointType MapLocalizing::get_last_point_type() {
  PointType ret;
  ret.x = transformTobeMapped[3];
  ret.y = transformTobeMapped[4];
  ret.z = transformTobeMapped[5];
  return ret;
}
PointTypePose MapLocalizing::get_last_point_type_pose() {
  PointTypePose ret;
  ret.roll = transformTobeMapped[0];
  ret.pitch = transformTobeMapped[1];
  ret.yaw = transformTobeMapped[2];
  ret.x = transformTobeMapped[3];
  ret.y = transformTobeMapped[4];
  ret.z = transformTobeMapped[5];
  return ret;
}
void MapLocalizing::update_local_map() {
  PointType last_location = get_last_point_type();
  vector<int> indices;
  vector<float> distances;
  kdtree_global_map_ds->setInputCloud(global_map_ds_ptr);
  kdtree_global_map_ds->radiusSearch(last_location, global_map_ds_search_radius, indices, distances, 0);
  last_local_map_ptr->clear();
  for(const int& index : indices) {
    (*last_local_map_ptr).push_back((*global_map_ds_ptr).points[index]);
  }
  ROS_INFO_STREAM("last_local_map size : " << last_local_map_ptr->size());
}

void MapLocalizing::localizing() {
  if(ndt.getInputTarget() == nullptr) {
    ROS_WARN_STREAM("No pcd map!");
    return;
  }
  // NDT between *global_map_ds_ptr and current input
  // Align clouds
  ndt.setInputSource(last_deskewed_ds_ptr);
  pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());

  Eigen::Matrix4f init_guess;
  if(init_ndt) {
    last_pose = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                        transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]).matrix();
    init_guess = last_pose;
    init_ndt = false;
  } else {
    init_guess = last_pose * delta_pose;
  }
  ndt.align(*unused_result, init_guess);
  // 检查icp是否收敛且得分是否满足要求
  if (!ndt.hasConverged() || ndt.getFitnessScore() > 0.3) {
    ROS_INFO_STREAM("NDT not convergence! Not update odometry");
    init_ndt = true;
//      return;
  }
  ROS_WARN_STREAM("NDT fitness score is too big. score : " << ndt.getFitnessScore());
  // Get pose transformation
  Eigen::Affine3f t_world_lidar;
  // 获得两个点云的变换矩阵结果
  t_world_lidar = ndt.getFinalTransformation();
  delta_pose = last_pose.inverse() * t_world_lidar.matrix();
  last_pose = t_world_lidar.matrix();
  pcl::getTranslationAndEulerAngles(t_world_lidar, transformTobeMapped[3], transformTobeMapped[4],
                                    transformTobeMapped[5], transformTobeMapped[0], transformTobeMapped[1],
                                    transformTobeMapped[2]);
//    ROS_INFO_STREAM("Last pose : " << ", x : " << transformTobeMapped[3] << ", y : " << transformTobeMapped[4] << ", z : "
//                                   << transformTobeMapped[5] << ", roll : " << transformTobeMapped[0] << ", pitch : "
//                                   << transformTobeMapped[1] << ", yaw : " << transformTobeMapped[2]);
  ROS_INFO_STREAM("last_pose : \n" << last_pose);
}
/**
 * @brief Obviously it is a global constant pcd map in the current system.
 */
void MapLocalizing::visualize_global_map_thread_func() {
  ros::Rate rate(0.1);
  while(ros::ok()) {
    rate.sleep();
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = world_coordinate_name;
    // global map
    PublishPointCloud2(global_map_pub, *global_map_ptr, header);
    PublishPointCloud2(global_map_ds_pub, *global_map_ds_ptr, header);
    // global ground truth trajectory
    PublishPointCloud2(global_gt_trajectory_pub, *global_gt_trajectory_ptr, header);
  }
}

} // namespace MapLocalization