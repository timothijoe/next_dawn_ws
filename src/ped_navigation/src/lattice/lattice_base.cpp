#include "lattice/lattice_base.h"

latticeBase::latticeBase()
{
  HZ = 10;    // 10.0
  MAX_ITERATION = 100;  // 100
  OPTIMIZATION_TOLERANCE = 0.1; // 0.1
  MAX_ACCELERATION = 1.0;
  TARGET_VELOCITY = 0.8;
  MAX_YAWRATE = 1.0;
  MAX_D_YAWRATE = 3.14;
  MAX_WHEEL_ANGULAR_VELOCITY = 11.6;
  WHEEL_RADIUS = 0.125;
  TREAD = 0.5;
  VERBOSE = false;
  ENABLE_CONTROL_SPACE_SAMPLING = false;

}


void latticeBase::getOdomTrajs(const vector<Trajectory> &local_trajs, vector<Trajectory> &odom_trajs)
{
  odom_trajs.clear();
  for(auto local_traj : local_trajs){
    bool is_collision = false;
    Trajectory odom_traj_;
    vector<Vector3d> trj;
    for(auto pose_: local_traj.trajectory){
      Vector3d odom_pose_ = localToOdom(pose_);
      if(_map_receiver.isObs(odom_pose_[0],odom_pose_[1])) is_collision = true;
      trj.push_back(odom_pose_);
    }
    odom_traj_.trajectory = trj;
    odom_traj_.velocities = local_traj.velocities;
    odom_traj_.angular_velocities = local_traj.angular_velocities;
    odom_traj_.is_collision = is_collision;
    odom_trajs.push_back(odom_traj_);
  }
}


bool latticeBase::generate_trajectories(const vector<Vector3d> &boundary_states, const double velocity, const double angular_velocity, const double target_velocity, vector<Trajectory> &trajectories)
{
  int count = 0;
  int trajectory_num = boundary_states.size();
  std::vector<MotionModelDiffDrive::Trajectory> trajectories_(trajectory_num);
  for(int i=0;i<trajectory_num;i++){
          Eigen::Vector3d boundary_state = boundary_states[i];
          TrajectoryGeneratorDiffDrive tg;
                  tg.set_verbose(VERBOSE);
                  tg.set_motion_param(MAX_YAWRATE, MAX_D_YAWRATE, MAX_ACCELERATION, MAX_WHEEL_ANGULAR_VELOCITY, \
                                      WHEEL_RADIUS, TREAD);
                  MotionModelDiffDrive::ControlParams output;
          double k0 = angular_velocity;
          ControlParams param;
          LookupTableUtils::get_optimized_param_from_lookup_table(lookup_table, boundary_state, velocity, k0, param);
          ControlParams init(VelocityParams(velocity, MAX_ACCELERATION, target_velocity, target_velocity, MAX_ACCELERATION),\
                             AngularVelocityParams(k0, param.omega.km, param.omega.kf, param.omega.sf));
          Trajectory trajectory;
          //double cost = tg.generate_optimized_trajectory(boundary_state, init, 1.0 / HZ, OPTIMIZATION_TOLERANCE * 2, MAX_ITERATION, output, trajectory);
          double cost = tg.generate_non_optimized_trajectory(boundary_state, init, 1.0 / HZ, OPTIMIZATION_TOLERANCE, MAX_ITERATION, output, trajectory);
          if(cost > 0){
            trajectories_[i] = trajectory;
            count++;}
  }
  for(auto it=trajectories_.begin();it!=trajectories_.end();){
      if(it->trajectory.size() == 0){
          it = trajectories_.erase(it);
      }else{
          ++it;
      }
  }
  if(ENABLE_CONTROL_SPACE_SAMPLING){
    int min_trajectory_size = trajectories_[0].trajectory.size();
    for(auto& traj : trajectories_){
      min_trajectory_size = std::min(min_trajectory_size, (int)traj.trajectory.size());
    }
    for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        MotionModelDiffDrive::Trajectory traj;
        MotionModelDiffDrive mmdd;
        mmdd.set_param(MAX_YAWRATE, MAX_D_YAWRATE, MAX_ACCELERATION, MAX_WHEEL_ANGULAR_VELOCITY, WHEEL_RADIUS, TREAD);
        MotionModelDiffDrive::State state(0, 0, 0, velocity, angular_velocity);
        traj.trajectory.emplace_back(Eigen::Vector3d(state.x, state.y, state.yaw));
        traj.velocities.emplace_back(state.v);
        traj.angular_velocities.emplace_back(state.omega);
        double velocity_ = velocity + (j - 1) * MAX_ACCELERATION / HZ;
        velocity_ = std::min(TARGET_VELOCITY, std::max(-TARGET_VELOCITY, velocity_));
        double omega = angular_velocity + (i - 1) * MAX_D_YAWRATE / HZ;
        omega = std::min(MAX_YAWRATE, std::max(omega, -MAX_YAWRATE));
        for(int j=0;j<min_trajectory_size;j++){
          mmdd.update(state, velocity_, omega, 1.0 / HZ, state);
          traj.trajectory.emplace_back(Eigen::Vector3d(state.x, state.y, state.yaw));
          traj.velocities.emplace_back(state.v);
          traj.angular_velocities.emplace_back(state.omega);
        }
        trajectories_.emplace_back(traj);
      }
    }
  }
  std::copy(trajectories_.begin(), trajectories_.end(), std::back_inserter(trajectories));
  if(trajectories.size() == 0){
      std::cout << "\033[91mERROR: no trajectory was generated\033[00m" << std::endl;
      return false;
  }
  return true;
}

bool latticeBase::isOccluded(double x_, double y_)
{
  Vector3d local_pos_(x_, y_, 0);
  Vector3d pos_ = localToOdom(local_pos_);
  return _map_receiver.isObs(pos_[0], pos_[1]);
}

double latticeBase::calculate_rpy_from_quat(geometry_msgs::Quaternion q)
{
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  double angle = std::atan2(siny_cosp, cosy_cosp);
  return angle;
}
