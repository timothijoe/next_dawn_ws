#include "point_sampler.h"

pointSampler::pointSampler(ros::NodeHandle nh)
{
  //_map_util_sub = nh.subscribe("/map_utils", 1, &pointSampler::update_map_util, this);
  //_ped_subscriber = nh.subscribe("/pedsim_simulator/simulated_agents", 1, &decisionMaker::pedestrian_callback, this);
  _robot_pose_sub = nh.subscribe("/pedsim_simulator/robot_position", 1, &pointSampler::robot_pose_callback, this);
  _timer = nh.createTimer(ros::Duration(0.4), &pointSampler::timer_callback, this);
  _path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("RRTstar_path_vis",1);
  _opt_path_pub = nh.advertise<visualization_msgs::Marker>("optimal_choice",1);
  _propose_pos_pub = nh.advertise<ped_navigation::p_opt_trajectory>("/optimal_traj_choice",1);
  _resetTargetPosServer = nh.advertiseService("/ped_navigation/reset_target_pos", &pointSampler::reset_target_pos_func,this);
  //_triangle_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/triangles",1);
  std::string LOOKUP_TABLE_FILE_NAME = "/home/tony-joe/river_spring/dell_ws/src/ped_navigation/lookup_table/lookup_table_try_v1.csv";
  load_lookup_table(LOOKUP_TABLE_FILE_NAME,lookup_table);

  _target_pos << 9, 8,0 * M_PI / 2;

  HZ = 10;    // 10.0
  MAX_ITERATION = 100;  // 100
  OPTIMIZATION_TOLERANCE = 10; // 0.1
  MAX_ACCELERATION = 1.0;
  TARGET_VELOCITY = 0.8;
  MAX_YAWRATE = 0.4;
  MAX_D_YAWRATE = 1.0;
  MAX_WHEEL_ANGULAR_VELOCITY = 11.6;
  WHEEL_RADIUS = 0.125;
  TREAD = 0.5;
  VERBOSE = false;
  ENABLE_CONTROL_SPACE_SAMPLING = false;
  tg.set_verbose(VERBOSE);
  tg.set_motion_param(MAX_YAWRATE, MAX_D_YAWRATE, MAX_ACCELERATION, MAX_WHEEL_ANGULAR_VELOCITY, \
                      WHEEL_RADIUS, TREAD);

}

void pointSampler::robot_pose_callback(const nav_msgs::Odometry &data)
{
  double x_z = data.pose.pose.position.x;
  double y_z = data.pose.pose.position.y;
  double yaw_z = calculate_rpy_from_quat(data.pose.pose.orientation);
  _robot_pos << x_z, y_z, yaw_z;
  _robot_pos_ready = true;
  _data_ready = _robot_pos_ready;

}

void pointSampler::timer_callback(const ros::TimerEvent &)
{
  if(!_data_ready){
    return;
  }
  vector<Trajectory> local_trajectories;
  generate_trajectories( 0.8, 0, TARGET_VELOCITY, local_trajectories);
  if(local_trajectories.empty()) return;
  vector<Trajectory> odom_trajectories;
  getOdomTrajs(local_trajectories, odom_trajectories);
  visOdomStateLatticeTraj(odom_trajectories);


}

void pointSampler::getOdomTrajs(const vector<Trajectory> &local_trajs, vector<Trajectory> &odom_trajs)
{
  odom_trajs.clear();
  for(auto local_traj : local_trajs){
    bool is_collision = false;
    Trajectory odom_traj_;
    vector<Vector3d> trj;
    //std::cout << "trajectory lens: " << local_traj.trajectory.size();
    for(auto pose_: local_traj.trajectory){
      Vector3d odom_pose_ = localToOdom(pose_);
      //if(_map_receiver.isObs(odom_pose_[0],odom_pose_[1])) is_collision = true;
      trj.push_back(odom_pose_);
    }
    odom_traj_.angular_velocities = local_traj.angular_velocities;
    odom_traj_.velocities = local_traj.velocities;
    odom_traj_.trajectory = trj;
    odom_traj_.is_collision = is_collision;
    odom_trajs.push_back(odom_traj_);
  }
}

double pointSampler::calculate_rpy_from_quat(geometry_msgs::Quaternion q)
{
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  double angle = std::atan2(siny_cosp, cosy_cosp);
  return angle;
}

bool pointSampler::generate_trajectories(const double velocity, const double angular_velocity, const double target_velocity, vector<Trajectory> &trajectories_)
{
  int count = 0;
  std::vector<MotionModelDiffDrive::Trajectory> trajectories;
  vector<MotionModelDiffDrive::ControlParams> output;
  double k0 = angular_velocity;
  vector<ControlParams> params;
  Vector3d local_pos = odomToLocal(_target_pos);
  LookupTableUtils::get_optimized_params_from_lookup_table(lookup_table, local_pos, velocity, k0, params);
  for(auto param: params){
    ControlParams init(VelocityParams(velocity, MAX_ACCELERATION, target_velocity, target_velocity, MAX_ACCELERATION),\
                       AngularVelocityParams(k0, param.omega.km, param.omega.kf, param.omega.sf));
    Trajectory trajectory;
    tg.set_motion_param(MAX_YAWRATE, MAX_D_YAWRATE, MAX_ACCELERATION, MAX_WHEEL_ANGULAR_VELOCITY, WHEEL_RADIUS, TREAD);
    double cost = tg.generate_trajectory_from_param(init, trajectory);
    //double cost = tg.generate_non_optimized_trajectory(_target_pos, init, 1.0 / HZ, OPTIMIZATION_TOLERANCE, MAX_ITERATION, output, trajectory);
    if(cost >0){trajectories.push_back(trajectory);}
  }

  for(auto it=trajectories.begin();it!=trajectories.end();){
      if(it->trajectory.size() == 0){
          it = trajectories.erase(it);
      }else{
          ++it;
      }
  }
  if(ENABLE_CONTROL_SPACE_SAMPLING){
    double min_trajectory_size = 30;
    for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        MotionModelDiffDrive::Trajectory traj;
        MotionModelDiffDrive mmdd;
        mmdd.set_param(MAX_YAWRATE, MAX_D_YAWRATE, MAX_ACCELERATION, MAX_WHEEL_ANGULAR_VELOCITY, WHEEL_RADIUS, TREAD);
        MotionModelDiffDrive::State state(0, 0, 0, 0.8, 0);
        traj.trajectory.emplace_back(Eigen::Vector3d(state.x, state.y, state.yaw));
        traj.velocities.emplace_back(state.v);
        traj.angular_velocities.emplace_back(state.omega);
        double velocity_ = velocity + (j - 1) * MAX_ACCELERATION / HZ;
        //velocity_ = std::min(TARGET_VELOCITY, std::max(-TARGET_VELOCITY, velocity_));
        velocity_ = std::min(TARGET_VELOCITY, std::max(0.0, velocity_));
        double omega = angular_velocity + (i - 1) * MAX_D_YAWRATE / HZ;
        omega = std::min(MAX_YAWRATE, std::max(omega, -MAX_YAWRATE));
        for(int j=0;j<min_trajectory_size;j++){
          mmdd.update(state, velocity_, omega, 1.0 / HZ, state);
          traj.trajectory.emplace_back(Eigen::Vector3d(state.x, state.y, state.yaw));
          traj.velocities.emplace_back(state.v);
          traj.angular_velocities.emplace_back(state.omega);
        }
        trajectories.emplace_back(traj);
      }
    }
  }
  std::copy(trajectories.begin(), trajectories.end(), std::back_inserter(trajectories_));
  if(trajectories.size() == 0){
      std::cout << "\033[91mERROR: no trajectory was generated\033[00m" << std::endl;
      return false;
  }

  return true;
}

void pointSampler::visOdomStateLatticeTraj(const vector<Trajectory> &trajectories)
{
  double _resolution = 0.2;
      visualization_msgs::MarkerArray  LineArray;
      int marker_id = 0;
      int count = 0;
      const int size = trajectories.size();
      for(;count<size;count++){
        visualization_msgs::Marker       Line;

        Line.header.frame_id = "odom";
        Line.header.stamp    = ros::Time::now();
        Line.ns              = "demo_node/TraLibrary";
        Line.action          = visualization_msgs::Marker::ADD;
        Line.pose.orientation.w = 1.0;
        Line.type            = visualization_msgs::Marker::LINE_STRIP;
        Line.scale.x         = _resolution/5;
        double color_r = (marker_id * 20 % 255) / 255;
        double color_g = (marker_id * 20 % 255) / 255;
         bool is_collision = trajectories[count].is_collision;
        if(is_collision == true){
          color_r = 1;
          color_g = 0;
        }
        else{
          color_r = 0;
          color_g = 1;
        }
        Line.color.r         = color_r;
        Line.color.g         = color_g;
        Line.color.b         = 0.5; // 1.0
        Line.color.a         = 0.4;
        Line.id = marker_id;
        Line.lifetime = ros::Duration(0.4);
        marker_id += 1;
        for(const auto& pose : trajectories[count].trajectory){
          //Vector3d odom_pose = localToOdom(pose);
          Vector3d odom_pose = pose;
          geometry_msgs::Point p;
          p.x = odom_pose(0);
          p.y = odom_pose(1);
          Line.points.push_back(p);
        }
        LineArray.markers.push_back(Line);
        //_path_vis_pub.publish(LineArray);
      }
      _path_vis_pub.publish(LineArray);
}

bool pointSampler::reset_target_pos_func(pedsim_srvs::ResetRobotPos::Request &req, pedsim_srvs::ResetRobotPos::Response &res)
{
  double r_x = req.robot_pos[0];
  double r_y = req.robot_pos[1];
  double r_theta = req.robot_pos[2] * M_PI / 180;
  _target_pos << r_x, r_y, r_theta;
  res.finished = true;
  std::cout << "value changed" << std::endl;
  return true;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "decisionMaker");
  const ros::NodeHandle& nh = ros::NodeHandle("~");
  pointSampler task_manager(nh);
  ros::spin();
  return 0;
}
