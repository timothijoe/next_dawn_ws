#ifndef TASKMANAGER_H
#define TASKMANAGER_H
#include "math.h"
#include <Eigen/Eigen>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ctime>
#include <chrono>
#include "flowsegments.h"

// ROS location
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int16.h>

#include "lattice/map_receiver.h"

#include "lattice/diff_drive_generator/motion_model_diff_drive.h"
#include "lattice/diff_drive_generator/trajectory_generator_diff_drive.h"
#include "lattice/lattice_lookup_table/lookup_table_utils.h"

//#include "graph/accessor.hpp"
#include "graph/delaunator.hpp"

#include "ped_navigation/m_density_map.h"
#include "ped_navigation/m_static_map.h"
#include "ped_navigation/m_velocity_map.h"
#include "ped_navigation/map_utils.h"
#include "ped_navigation/p_opt_trajectory.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <math.h>

using namespace std::chrono;
using namespace std;
using namespace Eigen;

typedef MotionModelDiffDrive::ControlParams ControlParams;
typedef MotionModelDiffDrive::Trajectory Trajectory;
typedef MotionModelDiffDrive::AngularVelocityParams AngularVelocityParams;
typedef MotionModelDiffDrive::VelocityParams VelocityParams;

enum metaTask{
  INTO_FLOW,
  OUT_OF_FLOW,
  CROSS_FLOW,
  ALONG_WITH_FLOW,
  AGAINST_WITH_FLOW,
  NORMAL
};
enum methodName{
  LATTICE_CHOICE,
  NORMALL
};
class scoreEvaluator{
  public:
  scoreEvaluator(){};

  double getTailScore(Trajectory & trajectory, FlowSegments& segs){
    switch(_task_type){
    case INTO_FLOW:
    {
      FlowSegment seg_;
      double dist_;
      int trajectory_size = trajectory.trajectory.size();
      Vector3d end_pose = trajectory.trajectory[trajectory_size -1];
      Vector2d end_position = end_pose.segment(0,2);
      segs.find_nearest_segment_and_dist(end_position, seg_, dist_);
      double score_raw = dist_ - seg_.width;
      double score = max(0.0, score_raw);
      return score;
    }
    default: return 0;
    }
  }
  metaTask _task_type = INTO_FLOW;
  FlowSegment target_segment;


};
class taskManager
{
public:

  taskManager(ros::NodeHandle nh);
  metaTask _task_type = INTO_FLOW;
  methodName _method_name = LATTICE_CHOICE;
  FlowSegments flow_segments = FlowSegments();
//  FlowSegment target_segment;
//  int current_seg_id = -1;
  scoreEvaluator score_evaluator = scoreEvaluator();

  void update_map_util(const ped_navigation::map_utilsPtr data);
  void setStaticCoords();
  void robot_pose_callback(const nav_msgs::Odometry& data);
  void timer_callback(const ros::TimerEvent &);
  void getOdomTrajs(const vector<Trajectory>& local_trajs, vector<Trajectory>& odom_trajs);
  bool isOccluded(double x_, double y_);
  double getDensity(double x_, double y_);
  vector<double> combine_coords();
Vector2d getVelocity(double x_, double y_);
  Vector3d localToOdom(Vector3d local_goal);
  Vector3d odomToLocal(Vector3d goal);
  double calculate_rpy_from_quat(geometry_msgs::Quaternion q);

  vector<Vector3d> getSampleList();
  bool generate_trajectories(const vector<Vector3d>& boundary_states, const double velocity, \
                             const double angular_velocity, const double target_velocity, vector<Trajectory>& trajectories);

  void visOdomStateLatticeTraj(const vector<Trajectory> &trajectories);
  void visOdomOptimalTraj(const Trajectory trajectory);
  double getTrajScore(Trajectory & trajectory);
//  int seekTargetSegId();
//  int seekTargetSeg();
//  bool updateCurId();
  Trajectory chooseBestTraj(vector<Trajectory> &trajectories);
  void pubProTraj(Trajectory traj_);
  double range_angle_PI(double angle);




  ros::Subscriber _map_util_sub;
  ros::Subscriber _robot_pose_sub;
    //ros::Subscriber _ped_subscriber;
  ros::Timer _timer;


  ros::Publisher _path_vis_pub;
  ros::Publisher _opt_path_pub;
  ros::Publisher _propose_pos_pub;
  ros::Publisher _triangle_vis_pub;


  Vector3d _robot_pos;
  vector<double> _ped_coords;
  vector<double> _static_coords;

  LookupTableUtils::LookupTable lookup_table;
  mapReceiver _map_receiver;
  //vector<single_ped_xvi> cur_peds_xvi;

  bool _robot_pos_ready = false;
  bool _map_rece_ready = false;
  bool _data_ready = false;

  double _sur_weight = 60;
  double _traj_len_weight = 0.03;
  double _human_eff_weight = 80;

  TrajectoryGeneratorDiffDrive tg;

  // state lattice parameters
  double HZ;
  int MAX_ITERATION ;
  double OPTIMIZATION_TOLERANCE;
  double TARGET_VELOCITY;
  double MAX_ACCELERATION;
  double MAX_YAWRATE;
  double MAX_D_YAWRATE;
  double MAX_WHEEL_ANGULAR_VELOCITY;
  double WHEEL_RADIUS;
  double TREAD;
  bool VERBOSE;
  bool ENABLE_CONTROL_SPACE_SAMPLING;


};


inline Vector3d taskManager::odomToLocal(Vector3d goal)
{
  Vector3d local_goal;
  double yaw_robot = _robot_pos[2];
  double delta_x = goal[0] - _robot_pos[0];
  double delta_y = goal[1] - _robot_pos[1];
  double delta_yaw = goal[2] - _robot_pos[2];
  local_goal[2] = atan2(sin(delta_yaw), cos(delta_yaw));
  local_goal[0] = delta_x * cos(yaw_robot) + delta_y * sin(yaw_robot);
  local_goal[1] = -delta_x * sin(yaw_robot) + delta_y * cos(yaw_robot);
  return local_goal;
}

inline double taskManager::range_angle_PI(double angle)
{
  if (angle > M_PI)
    angle -= 2 * M_PI;
  else if (angle < -M_PI)
    angle += 2 * M_PI;
  return angle;
}


inline Vector3d taskManager::localToOdom(Vector3d local_goal)
{
  Vector3d odom_goal;
  double delta_length = local_goal.segment(0, 2).norm();
  double delta_angle = atan2(local_goal[1],local_goal[0]);
  double total_angle = delta_angle + _robot_pos[2];
  odom_goal[0] = delta_length * cos(total_angle) + _robot_pos[0];
  odom_goal[1] = delta_length * sin(total_angle) + _robot_pos[1];
  double yaw_z = local_goal[2] + _robot_pos[2];
  odom_goal[2] = atan2(sin(yaw_z), cos(yaw_z));
  return odom_goal;
}











#endif // TASKMANAGER_H
