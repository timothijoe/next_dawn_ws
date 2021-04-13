#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H
#include <ros/console.h>
#include <ros/ros.h>
#include <functional>
#include <iostream>
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/Eva_Info.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <math.h>
#include <bits/stdc++.h>
#include <nav_msgs/Odometry.h>
#include "map_generator/ped_xvi.h"
#include "map_generator/quad_tree.h"
using namespace std;
using namespace Eigen;

class CollisionChecker
{
public:
  CollisionChecker(ros::NodeHandle nh);
  void pedestrian_callback(const pedsim_msgs::AgentStatesPtr data);
  void timer_callback(const ros::TimerEvent &);
  void update_shutdown_parameter(const ros::TimerEvent &);
  void robot_pose_callback(const nav_msgs::Odometry& data);
  double calculate_rpy_from_quat(geometry_msgs::Quaternion q);
  void post_collision_function();
private:
  ros::Subscriber _ped_subscriber;
  ros::Subscriber robot_pose_sub;
  ros::ServiceClient stop_client;
  ros::ServiceClient robot_stop_continue_client;
  ros::Timer _timer;
  ros::Timer _update_shutdown_timer_;

  quadTree quad_tree = quadTree();

  bool collision_occured_label = false;
  bool data_ready_label = false;
  bool exec_ready_label = true;
  bool system_shutdown_label = false;

  // if this label start, the system will not be stop, just remind you that we have faced collision.
  bool ignore_collision_label = false;
  double dist_safe = 0.6;
  int collision_mem_id = -1;

  Vector3d robot_position;
  vector<single_ped_xvi> cur_peds_xvi;

};

#endif // COLLISION_CHECKER_H
