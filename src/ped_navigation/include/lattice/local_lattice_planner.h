#ifndef LOCAL_LATTICE_PLANNER_H
#define LOCAL_LATTICE_PLANNER_H

// ROS location
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int16.h>

// io stream
#include <sstream>
#include <fstream>
#include <iostream>
#include <ctime>
#include <chrono>

#include "lattice/lattice_base.h"

#include "math.h"


class localLatticePlanner : public latticeBase
{
public:
  localLatticePlanner(ros::NodeHandle nh);
  void update_map_callback(const ped_navigation::map_utilsPtr data);
  void robot_pose_callback(const nav_msgs::Odometry& data);
  void proposed_traj_callback(const ped_navigation::p_opt_trajectory& pro_trajectory);
  void timer_callback(const ros::TimerEvent &);
  void timer_pub_vel(const ros::TimerEvent &);
  void get_velocity();
  vector<Vector3d> getSampleList();
  void visOdomStateLatticeTraj(const vector<Trajectory> &trajectories);
  void visOdomOptimalTraj(const Trajectory trajectory);
  Trajectory chooseBestTraj(vector<Trajectory> &trajectories);
  double normalizeRad(double val_);
  void getTrajScore(Trajectory & trajectory);


private:
  vector<double> _ped_coords;
  vector<double> _static_coords;
  double _start_time;
  Trajectory _opt_traj;


  ros::Subscriber _map_util_sub;
  ros::Subscriber _robot_pose_sub;
  ros::Subscriber _pro_traj_sub;
  ros::Timer _timer;

  ros::Publisher _path_vis_pub;
  ros::Publisher _opt_path_pub;
  ros::Publisher _velocity_pub;

  Vector3d _proposed_target;


  double _sur_weight = 60;
  double _rad_vary_weight = 0.5;
  double _cur_vel;
  double _cur_twist;

  bool _map_rece_ready = false;
  bool _robot_rece_ready = false;
  bool _pro_traj_ready = false;
  bool _data_ready = false;
  bool _all_traj_fail = false;
};

#endif // LOCAL_LATTICE_PLANNER_H
