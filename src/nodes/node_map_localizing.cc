#include "map_localizing.h"

using namespace std;
using namespace MapLocalization;

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_localizing");

  ROS_INFO("\033[1;32m----> Map Localizing Started.\033[0m");

  MapLocalizing map_localizing;
  std::thread visualize_global_map_thread(&MapLocalizing::visualize_global_map_thread_func, &map_localizing);

  ros::spin();
  visualize_global_map_thread.join();
  return 0;
}