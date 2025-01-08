#include <ros/ros.h>
#include "fast_planner_sdf_map/sdf_map.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "sdf_map_node");
  ros::NodeHandle nh("~");

  SDFMap::Ptr sdf_map(new SDFMap());
  sdf_map->initMap(nh);

  ros::spin();

  return 0;
}

