#include "taskmanager.h"
#include <ros/package.h>
taskManager::taskManager(ros::NodeHandle nh)
{
  _map_util_sub = nh.subscribe("/map_utils", 1, &taskManager::update_map_util, this);
  //_ped_subscriber = nh.subscribe("/pedsim_simulator/simulated_agents", 1, &decisionMaker::pedestrian_callback, this);
  _robot_pose_sub = nh.subscribe("/pedsim_simulator/robot_position", 1, &taskManager::robot_pose_callback, this);
  _timer = nh.createTimer(ros::Duration(0.4), &taskManager::timer_callback, this);
  _path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("RRTstar_path_vis",1);
  _opt_path_pub = nh.advertise<visualization_msgs::Marker>("optimal_choice",1);
  _propose_pos_pub = nh.advertise<ped_navigation::p_opt_trajectory>("/optimal_traj_choice",1);
  _triangle_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/triangles",1);
  setStaticCoords();
  std::string map_generator_pack = ros::package::getPath("ped_navigation");
  std::string LOOKUP_TABLE_FILE_NAME = map_generator_pack + "/lookup_table/lookup_table.csv";
  //std::string LOOKUP_TABLE_FILE_NAME = "/home/tony-joe/river_spring/dell_ws/src/ped_navigation/lookup_table/lookup_table.csv";
  load_lookup_table(LOOKUP_TABLE_FILE_NAME,lookup_table);

      HZ = 10;    // 10.0
      MAX_ITERATION = 100;  // 100
      OPTIMIZATION_TOLERANCE = 10; // 0.1
      MAX_ACCELERATION = 1.0;
      TARGET_VELOCITY = 0.8;
      MAX_YAWRATE = 1.0;
      MAX_D_YAWRATE = 3.14;
      MAX_WHEEL_ANGULAR_VELOCITY = 11.6;
      WHEEL_RADIUS = 0.125;
      TREAD = 0.5;
      VERBOSE = false;
      ENABLE_CONTROL_SPACE_SAMPLING = true;
      tg.set_verbose(VERBOSE);
      tg.set_motion_param(MAX_YAWRATE, MAX_D_YAWRATE, MAX_ACCELERATION, MAX_WHEEL_ANGULAR_VELOCITY, \
                          WHEEL_RADIUS, TREAD);
//      current_seg_id = -1;
//      int m = seekTargetSegId();
//      target_segment = flow_segments.segments[m+1];
      //tg.setDecisionMaker(this);
}

void taskManager::update_map_util(const ped_navigation::map_utilsPtr data)
{
  if(!_map_rece_ready)
  {
    Vector2d origin_( data->origin.x, data->origin.y);
    Vector2d resolution_(data->resolutions.x, data->resolutions.y);
    Vector2i map_size_(data->size_x, data->size_y);
    _map_receiver.setMapSize(resolution_, origin_, map_size_);
  }
  _map_receiver.setStaticMap(data->static_map.map);
  _map_receiver.setClusterMap(data->cluster_map.map);
  _map_receiver.setDensityMap(data->density_map.map);
  _map_receiver.setVelMap(data->velocity_map.map_x, data->velocity_map.map_y);
  _ped_coords.clear();
  for(auto ped_info: data->peds_info){
    _ped_coords.push_back(ped_info.x);
    _ped_coords.push_back(ped_info.y);
  }

  _map_rece_ready = true;
  _data_ready = _map_rece_ready && _robot_pos_ready;
}

void taskManager::setStaticCoords()
{
  _static_coords.clear();
  std::vector<Vector2d> static_obs_list = {
    {-5, -5}, { 0, -5}, {0, -5}, {10, -5}, {15, -5},{ 20, -5}, {25, -5},
    {-5, 0}, { 0, 0}, {5, 0}, {10, 0}, {15, 0},{ 20, 0}, {25, 0},
    {-5, 5}, { 0, 5}, {5, 5}, {10, 5}, {15, 5},{ 20, 5}, {25, 5},
    {-5, 10}, { 0, 10}, {5, 10}, {10, 10}, {15, 10},{ 20, 10}, {25, 10},
    {-5, 15}, { 0, 15}, {5, 15}, {10, 15}, {15, 15},{ 20, 15}, {25, 15},
//    {-5, 20}, { 0, 20}, {5, 20}, {10, 20}, {15, 20},{ 20, 20}, {25, 20},
//    {-5, 25}, { 0, 25}, {5, 25}, {10, 25}, {15, 25},{ 20, 25}, {25, 25}
  };
  for(auto static_ob_coord: static_obs_list){
  _static_coords.push_back(static_ob_coord[0]);
  _static_coords.push_back(static_ob_coord[1]); }
}

void taskManager::robot_pose_callback(const nav_msgs::Odometry &data)
{
  double x_z = data.pose.pose.position.x;
  double y_z = data.pose.pose.position.y;
  double yaw_z = calculate_rpy_from_quat(data.pose.pose.orientation);
  _robot_pos << x_z, y_z, yaw_z;
  _robot_pos_ready = true;
  _data_ready = _map_rece_ready && _robot_pos_ready;

}

void taskManager::timer_callback(const ros::TimerEvent &)
{
  if(!_data_ready) return;
  vector<Vector3d> candidate_samples_ = getSampleList();
  vector<Trajectory> local_trajectories;
  generate_trajectories(candidate_samples_, 0, 0, TARGET_VELOCITY, local_trajectories);
  if(local_trajectories.empty()) return;
  vector<Trajectory> odom_trajectories;
  getOdomTrajs(local_trajectories, odom_trajectories);
  visOdomStateLatticeTraj(odom_trajectories);
  Trajectory traj = chooseBestTraj(odom_trajectories);
  visOdomOptimalTraj(traj);
  pubProTraj(traj);

//  // here are new
//  updateCurId();
//  int t_id = seekTargetSegId();
//  std::cout << "current target id:" << t_id << std::endl;
//  target_segment = flow_segments.segments[t_id];
}

void taskManager::getOdomTrajs(const vector<Trajectory> &local_trajs, vector<Trajectory> &odom_trajs)
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
    odom_traj_.angular_velocities = local_traj.angular_velocities;
    odom_traj_.velocities = local_traj.velocities;
    odom_traj_.trajectory = trj;
    odom_traj_.is_collision = is_collision;
    odom_trajs.push_back(odom_traj_);
  }
}

bool taskManager::isOccluded(double x_, double y_)
{
  Vector3d local_pos_(x_, y_, 0);
  Vector3d pos_ = localToOdom(local_pos_);
  return _map_receiver.isObs(pos_[0], pos_[1]);

}

double taskManager::getDensity(double x_, double y_)
{
  Vector3d local_pos_(x_, y_, 0);
  Vector3d pos_ = localToOdom(local_pos_);
  return _map_receiver.getDensity(pos_[0], pos_[1]);

}

vector<double> taskManager::combine_coords()
{
  vector<double> coords = _static_coords;
  for(auto co: _ped_coords){
    coords.push_back(co);
  }
  coords.push_back(_robot_pos[0]);
  coords.push_back(_robot_pos[1]);
  return coords;
}
Vector2d taskManager::getVelocity(double x_, double y_)
{
  Vector3d local_pos_(x_, y_, 0);
  Vector3d pos_ = localToOdom(local_pos_);
  return _map_receiver.getVelocity(pos_[0], pos_[1]);
}
//getSampleList
double taskManager::calculate_rpy_from_quat(geometry_msgs::Quaternion q)
{
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  double angle = std::atan2(siny_cosp, cosy_cosp);
  return angle;
}

vector<Vector3d> taskManager::getSampleList()
{
  vector<Vector3d> candidate_3d_samples_;
   vector<double> coords = combine_coords();
   vector<double> speeds(coords.size(),0);
   vector<double> angles(coords.size(),0);
   auto graph = delaunator::Delaunator(coords, speeds, angles);
   auto tri_size = graph.triangles.size();
   vector<Vector2d> candidate_samples;
   for(int i = 0; i<tri_size; i+=3){
     Vector2d v1, v2, v3;
     v1 << graph.coords[2 * graph.triangles[i]], graph.coords[2 * graph.triangles[i] + 1];
     v2 << graph.coords[2 * graph.triangles[i + 1]], graph.coords[2 * graph.triangles[i + 1] + 1];
     v3 << graph.coords[2 * graph.triangles[i + 2]], graph.coords[2 * graph.triangles[i + 2] + 1];
     Vector2d centre_;
     centre_ << (v1[0] + v2[0] + v3[0])/3, (v1[1] + v2[1] + v3[1])/3;
     candidate_samples.push_back(centre_);
   }



   // visualize triangles
   visualization_msgs::MarkerArray triangles_array;
   for(int i = 0; i < tri_size; i+=3){
     visualization_msgs::Marker triangle;
     triangle.type = visualization_msgs::Marker::LINE_STRIP;
     geometry_msgs::Point m1, m2, m3;
     m1.x = graph.coords[2 * graph.triangles[i]];
     m1.y = graph.coords[2 * graph.triangles[i] + 1];
     m2.x = graph.coords[2 * graph.triangles[i + 1]];
     m2.y = graph.coords[2 * graph.triangles[i + 1] + 1];
     m3.x = graph.coords[2 * graph.triangles[i + 2]];
     m3.y = graph.coords[2 * graph.triangles[i + 2] + 1];
     triangle.header.frame_id = "odom";
     triangle.header.stamp = ros::Time::now();
     triangle.action = visualization_msgs::Marker::ADD;
     triangle.scale.x = 0.1;
     triangle.color.a = 0.4;
     triangle.color.r = 1;
     triangle.color.g = 0.4;
     triangle.color.b = 0.4;
     triangle.id = i;
     triangle.points.push_back(m1);
     triangle.points.push_back(m2);
     triangle.points.push_back(m3);
     triangle.points.push_back(m1);
     triangle.lifetime =  ros::Duration(0.4);
     triangles_array.markers.push_back(triangle);
   }
   _triangle_vis_pub.publish(triangles_array);


   for(auto sample: candidate_samples){

     auto dist = (sample - _robot_pos.segment(0,2)).norm();
     if(dist < 5){
       if(_map_receiver.getCluster(sample[0], sample[1]) == 1){
         Vector2d vel = _map_receiver.getVelocity(sample[0], sample[1]);
         if((abs(vel[0])<0.000001)&&(abs(vel[1])<0.000001)){continue;}
         double angle = std::atan2(vel[1], vel[0]);
         for(int i = -1; i <= 1; i++){
           double angle_i = 3.14/4 * i +angle;
           Vector3d pos(sample[0] + 0.4* vel[0], sample[1] + 0.4*vel[1], angle_i);
           Vector3d local_pos = odomToLocal(pos);
           candidate_3d_samples_.push_back(local_pos);
         //std::cout << "candidate coordinates: (" << pos[0] <<" , " <<pos[1]<< ")" << std::endl;
         }

         }
     }
   }
   return candidate_3d_samples_;
}

bool taskManager::generate_trajectories(const vector<Vector3d> &boundary_states, const double velocity, const double angular_velocity, const double target_velocity, vector<Trajectory> &trajectories)
{
  int count = 0;
  int trajectory_num = boundary_states.size();
  std::vector<MotionModelDiffDrive::Trajectory> trajectories_(trajectory_num);
  for(int i=0;i<trajectory_num;i++){
          Eigen::Vector3d boundary_state = boundary_states[i];
//          TrajectoryGeneratorDiffDrive tg;
//                  tg.set_verbose(VERBOSE);
//                  tg.set_motion_param(MAX_YAWRATE, MAX_D_YAWRATE, MAX_ACCELERATION, MAX_WHEEL_ANGULAR_VELOCITY, \
//       update_map_util                               WHEEL_RADIUS, TREAD);
//                  tg.setDecisionMaker(this);
                  MotionModelDiffDrive::ControlParams output;
          double k0 = angular_velocity;
          ControlParams param;
          LookupTableUtils::get_optimized_param_from_lookup_table(lookup_table, boundary_state, velocity, k0, param);
          ControlParams init(VelocityParams(velocity, MAX_ACCELERATION, target_velocity, target_velocity, MAX_ACCELERATION),\
                             AngularVelocityParams(k0, param.omega.km, param.omega.kf, param.omega.sf));
          Trajectory trajectory;
          //double cost = tg.generate_optimized_trajectory(boundary_state, init, 1.0 / HZ, OPTIMIZATION_TOLERANCE, MAX_ITERATION, output, trajectory);
          double cost = tg.generate_non_optimized_trajectory(boundary_state, init, 1.0 / HZ, OPTIMIZATION_TOLERANCE, MAX_ITERATION, output, trajectory);
          //double cost = tg.generate_hm_optimized_trajectory(boundary_state, init, 1.0 / HZ, OPTIMIZATION_TOLERANCE, MAX_ITERATION, output, trajectory);
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
//    for(auto& traj : trajectories_){
//      min_trajectory_size = std::min(min_trajectory_size, (int)traj.trajectory.size());
//    }
    min_trajectory_size = 30;
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

void taskManager::visOdomStateLatticeTraj(const vector<Trajectory> &trajectories)
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
        Line.color.a         = 0.2;
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

void taskManager::visOdomOptimalTraj(const Trajectory trajectory)
{
  visualization_msgs::Marker       Line;
  double resolution = 0.2;
  Line.header.frame_id = "odom";
  Line.header.stamp    = ros::Time::now();
  Line.ns              = "demo_node/TraLibrary";
  Line.action          = visualization_msgs::Marker::ADD;
  Line.pose.orientation.w = 1.0;
  Line.type            = visualization_msgs::Marker::LINE_STRIP;
  Line.scale.x         = resolution/5;


  Line.color.r         = 0.3;
  Line.color.g         = 0.3;
  Line.color.b         = 0.5; // 1.0
  Line.color.a         = 1.0;
  Line.id = 100;
  Line.lifetime = ros::Duration(0.4);
  for(const auto& pose : trajectory.trajectory){
    //Vector3d odom_pose = localToOdom(pose);
    Vector3d odom_pose = pose;
    geometry_msgs::Point p;
    p.x = odom_pose(0);
    p.y = odom_pose(1);
    Line.points.push_back(p);
  }
  _opt_path_pub.publish(Line);
}

double taskManager::getTrajScore(Trajectory &trajectory)
{
  double _dense_weight = 60;
  double _traj_len_weight = 0.03;
  double _twist_weight = 1;
  double _tail_weight = 3;
  double _yaw_rate = 2;

  int traj_len = trajectory.trajectory.size();

  Vector3d end_pos = trajectory.trajectory[traj_len-1];
  double tail_score = score_evaluator.getTailScore(trajectory, flow_segments);
  double end_pos_density = _map_receiver.getDensity(end_pos[0], end_pos[1]);
  double min_distance = end_pos_density;
  double avg_dense = 0;
  double avg_twist;
  double min_angle, max_angle;
  for(auto pos_ : trajectory.trajectory){
    double cur_dense = _map_receiver.getDensity(pos_[0], pos_[1]);
    if(cur_dense < min_distance){min_distance = cur_dense;}
    avg_dense += cur_dense;
    double n_angle = range_angle_PI(pos_[2]);
    if(min_angle > n_angle){min_angle = n_angle;}
    if(max_angle < n_angle){max_angle = n_angle;}
  }
  for(double i = -0.3; i <= 0.3; i+=0.3){
    for(double j = -0.3; j <= 0.3; j+=0.3){
      Vector2d pos_(end_pos[0]+i, end_pos[1]+j);
      double temp_dist = _map_receiver.getDensity(pos_[0], pos_[1]);
      if(temp_dist < min_distance){min_distance = temp_dist;}
    }
  }
  avg_dense = avg_dense / traj_len;
  for(auto twist_: trajectory.angular_velocities){
    avg_twist += abs(twist_);
  }
  avg_twist /= traj_len;
  double distance_cost = max(0.2, 1 - min_distance);
  double total_score = _dense_weight*distance_cost + _yaw_rate * (max_angle - min_angle) + _traj_len_weight*double(traj_len) + _tail_weight*tail_score;
  std::cout << "dense weight: " << _dense_weight*avg_dense  << std::endl;
  std::cout << "max angle: " << max_angle << " , and min angle: " << min_angle << std::endl;
  std::cout << "twist weight: " <<_yaw_rate * (max_angle - min_angle)  << std::endl;
  std::cout << "len weight: " << _traj_len_weight*double(traj_len) << std::endl;
  std::cout << "tail weight: " << _tail_weight*tail_score << std::endl;
  return total_score;
}



Trajectory taskManager::chooseBestTraj(vector<Trajectory> &trajectories)
{
  int traj_num = trajectories.size();
  double min_score = DBL_MAX;
  int traj_id = 0;
  for(int i = 0; i < traj_num; i++){
    double cur_score = getTrajScore(trajectories[i]);
    if(cur_score <= min_score){
      min_score = cur_score;
      traj_id = i;}
  }
  //std::cout << traj_id << "hi"<< std::endl;
  return trajectories[traj_id];

}

void taskManager::pubProTraj(Trajectory traj_)
{
  ped_navigation::p_opt_trajectory p_traj;
  int traj_len = traj_.trajectory.size();
  Vector3d final_pos_ = traj_.trajectory[traj_len-1];
  p_traj.final_pos[0] = final_pos_[0];
  p_traj.final_pos[1] = final_pos_[1];
  p_traj.final_pos[2] = final_pos_[2];
  vector<double> coords;
  for(auto pos_ : traj_.trajectory){
    coords.push_back(pos_[0]);
    coords.push_back(pos_[1]);
  }
  p_traj.opt_trajectory =coords;
  _propose_pos_pub.publish(p_traj);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "decisionMaker");
  const ros::NodeHandle& nh = ros::NodeHandle("~");
  taskManager task_manager(nh);
  ros::spin();
  return 0;
}
