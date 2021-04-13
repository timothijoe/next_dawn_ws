#ifndef MAP_GENERATOR_H
#define MAP_GENERATOR_H

// ROS location
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int16.h>
// pedsim include
#include <pedsim_msgs/AgentStates.h>

// io stream
#include <sstream>
#include <fstream>
#include <iostream>
#include <ctime>
#include <chrono>

// include of this package
#include "map_generator/ped_xvi.h"
#include "map_generator/quad_tree.h"
#include "map_generator/coherent_filter.h"
#include "map_utils/static_map.h"
#include "map_utils/density_map.h"
#include "map_utils/velocity_map.h"
#include "map_utils/base_map.h"
#include "map_utils/cluster_map.h"
#include "ped_navigation/m_density_map.h"
#include "ped_navigation/m_static_map.h"
#include "ped_navigation/m_velocity_map.h"
#include "ped_navigation/m_cluster_map.h"
#include "ped_navigation/map_utils.h"

using namespace std::chrono;
class mapGenerator
{
public:
  mapGenerator(ros::NodeHandle nh);
  void pedestrian_callback(const pedsim_msgs::AgentStatesPtr data);
  void timer_callback(const ros::TimerEvent &);
  void robot_pose_callback(const nav_msgs::Odometry& data);
  void updateMap();
  void setObsByPed();
  void testUpdateMap();
  double calcGaussian(double x, double h = 1);
  ped_navigation::map_utils preparePubMap();
  void publishMap();

private:
  staticMap _static_map;
  densityMap _density_map;
  velocityMap _velocity_map;
  clusterMap _cluster_map;
  quadTree _quad_tree;
  vector<single_ped_xvi> cur_peds_xvi;

  ros::Subscriber _ped_subscriber;
  ros::Subscriber _robot_pose_sub;
  ros::Publisher _map_utils_pub;
  ros::Timer _timer;

  bool executing_;
  bool data_ready_;
  bool draw_density_label = true;
  bool draw_velocity_label;
  int GLX_SIZE, GLY_SIZE;

};

#endif // MAP_GENERATOR_H
