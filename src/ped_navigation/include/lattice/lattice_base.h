#ifndef LATTICE_BASE_H
#define LATTICE_BASE_H

// io stream
#include <sstream>
#include <fstream>
#include <iostream>
#include <ctime>
#include <chrono>

#include "lattice/map_receiver.h"

#include "lattice/diff_drive_generator/motion_model_diff_drive.h"
#include "lattice/diff_drive_generator/trajectory_generator_diff_drive.h"
#include "lattice/lattice_lookup_table/lookup_table_utils.h"
#include "geometry_msgs/Quaternion.h"

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
class latticeBase
{
public:
  latticeBase();
  void getOdomTrajs(const vector<Trajectory>& local_trajs, vector<Trajectory>& odom_trajs);
  bool generate_trajectories(const vector<Vector3d>& boundary_states, const double velocity, \
                             const double angular_velocity, const double target_velocity, vector<Trajectory>& trajectories);
  bool isOccluded(double x_, double y_);
  inline Vector3d localToOdom(Vector3d local_goal);
  inline Vector3d odomToLocal(Vector3d goal);
  double calculate_rpy_from_quat(geometry_msgs::Quaternion q);

  // lattice planner element funciton
  virtual vector<Vector3d> getSampleList() = 0;
  virtual void visOdomStateLatticeTraj(const vector<Trajectory> &trajectories) = 0;
  virtual void visOdomOptimalTraj(const Trajectory trajectory) = 0;
  //virtual double getTrajScore(Trajectory & trajectory) = 0;
  //virtual Trajectory chooseBestTraj(vector<Trajectory> &trajectories) = 0;

protected:
  Vector3d _robot_pos;
  mapReceiver _map_receiver;

  LookupTableUtils::LookupTable lookup_table;

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

inline Vector3d latticeBase::localToOdom(Vector3d local_goal)
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

inline Vector3d latticeBase::odomToLocal(Vector3d goal)
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

#endif // LATTICE_BASE_H
