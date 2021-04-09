#ifndef EVALUATOR_H
#define EVALUATOR_H
// ROS location
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int16.h>

#include "evaluation/collision_checker.h"
#include "evaluation/success_checker.h"

#include "ped_navigation/m_density_map.h"
#include "ped_navigation/m_static_map.h"
#include "ped_navigation/m_velocity_map.h"
#include "ped_navigation/map_utils.h"
#include "evaluation/evaluation_sending.h"


class evaluator
{
public:
  evaluator(ros::NodeHandle nh);
  void update_map_util(const ped_navigation::map_utilsPtr data);
  void robot_pose_callback(const nav_msgs::Odometry& data);
  void timer_callback(const ros::TimerEvent &);
  double calculate_rpy_from_quat(geometry_msgs::Quaternion q);

private:
  collisionChecker _collision_checker;
  successChecker   _success_checker;

  bool _taskCompleteLabel = false;
  bool _collisionLabel = false;
  bool _TASKSTOPLABEL = false;
  bool data_ready;

  ros::Subscriber _map_util_sub;
  ros::Subscriber _robot_pose_sub;
  ros::Timer _timer;
  ros::Publisher _evaluation_pub;

};

#endif // EVALUATOR_H
