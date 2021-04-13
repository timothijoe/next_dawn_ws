#include "lattice/local_lattice_planner.h"
#include <ros/package.h>
localLatticePlanner::localLatticePlanner(ros::NodeHandle nh)
{
  _map_util_sub = nh.subscribe("/map_utils", 1, &localLatticePlanner::update_map_callback, this);
  _robot_pose_sub = nh.subscribe("/pedsim_simulator/robot_position", 1, &localLatticePlanner::robot_pose_callback, this);
  _pro_traj_sub = nh.subscribe("/optimal_traj_choice", 1, &localLatticePlanner::proposed_traj_callback, this);
  _timer = nh.createTimer(ros::Duration(0.1), &localLatticePlanner::timer_callback, this);
  _path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("local_paths_vis",1);
  _opt_path_pub = nh.advertise<visualization_msgs::Marker>("optimal_local_choice",1);
  _velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  TARGET_VELOCITY = 1.2;
  //std::string LOOKUP_TABLE_FILE_NAME = "/home/tony-joe/river_spring/dell_ws/src/ped_navigation/lookup_table/lookup_table.csv";
  std::string map_generator_pack = ros::package::getPath("ped_navigation");
  std::string LOOKUP_TABLE_FILE_NAME = map_generator_pack + "/lookup_table/lookup_table.csv";
  load_lookup_table(LOOKUP_TABLE_FILE_NAME,lookup_table);
}

void localLatticePlanner::update_map_callback(const ped_navigation::map_utilsPtr data)
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
}

void localLatticePlanner::robot_pose_callback(const nav_msgs::Odometry &data)
{
  double x_z = data.pose.pose.position.x;
  double y_z = data.pose.pose.position.y;
  double yaw_z = calculate_rpy_from_quat(data.pose.pose.orientation);
  _robot_pos << x_z, y_z, yaw_z;
  _robot_rece_ready = true;
  _data_ready = _map_rece_ready && _robot_rece_ready && _pro_traj_ready;
}

void localLatticePlanner::proposed_traj_callback(const ped_navigation::p_opt_trajectory &pro_trajectory)
{
 auto m = pro_trajectory.final_pos;
  _proposed_target << m[0], m[1], m[2];
  _pro_traj_ready = true;
}

void localLatticePlanner::timer_callback(const ros::TimerEvent &)
{
  if(!_data_ready) return;
  _start_time = ros::Time::now().toSec();
  vector<Vector3d> candidate_samples_ = getSampleList();
  vector<Trajectory> local_trajectories;
  generate_trajectories(candidate_samples_, _cur_vel, _cur_twist, TARGET_VELOCITY, local_trajectories);
  if(local_trajectories.empty()) return;
  vector<Trajectory> odom_trajectories;
  getOdomTrajs(local_trajectories, odom_trajectories);
  visOdomStateLatticeTraj(odom_trajectories);
  _opt_traj = chooseBestTraj(odom_trajectories);
  visOdomOptimalTraj(_opt_traj);
  get_velocity();

}

void localLatticePlanner::timer_pub_vel(const ros::TimerEvent &)
{


}

void localLatticePlanner::get_velocity()
{
  int CONTROL_DELAY = 4;
  geometry_msgs::Twist cmd_vel;
  HZ = 10;
  double calculation_time = ros::Time::now().toSec() - _start_time;
//  int delayed_control_index = std::min(std::ceil(calculation_time * HZ) + CONTROL_DELAY, \
//                                       (double)_opt_traj.trajectory.size());
  int delayed_control_index = std::min(int(std::ceil(calculation_time * HZ)) + CONTROL_DELAY, \
                                       (int)_opt_traj.trajectory.size());
  std::cout << "delayed control index: "<< delayed_control_index << std::endl;
  cmd_vel.linear.x = _opt_traj.velocities[delayed_control_index];
  cmd_vel.angular.z = _opt_traj.angular_velocities[delayed_control_index];
  _cur_vel = cmd_vel.linear.x;
  _cur_twist = cmd_vel.angular.z;
  std::cout << "number of angular velocities: "<< _opt_traj.angular_velocities.size() << std::endl;
  for(int i = 0; i < _opt_traj.angular_velocities.size(); i++){
    std::cout << "i: " << i << " with " << _opt_traj.angular_velocities[i] << std::endl;
  }
  _velocity_pub.publish(cmd_vel);
}

vector<Vector3d> localLatticePlanner::getSampleList()
{
  std::vector<Vector2d> sample_trans_list = {
    { 0, 0}, {0, 0.4}, {0.28, 0.28}, {0.4, 0},
    { 0.28, -0.28 }, {0, -0.4}, { -0.28, -0.28}, { -0.4, 0 }, { -0.28, 0.28 }};
  vector<Vector3d> sample_list;
  Vector3d sap_target;
  Vector2d bas_pos(_proposed_target[0], _proposed_target[1]);
  Vector2d sap_pos;
  for(auto sample_pos: sample_trans_list){
    sap_pos << bas_pos[0] + sample_pos[0], bas_pos[1] + sample_pos[1];
    Vector2d vel = _map_receiver.getVelocity(sap_pos[0], sap_pos[1]);
    double angle = std::atan2(vel[1], vel[0]);
    sap_target << sap_pos[0], sap_pos[1], angle;
    Vector3d local_pos = odomToLocal(sap_target);
    sample_list.push_back(local_pos);
  }
  return sample_list;
}

void localLatticePlanner::visOdomStateLatticeTraj(const vector<Trajectory> &trajectories)
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
        Line.color.b         = 0.7; // 1.0
        Line.color.a         = 0.6;
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

void localLatticePlanner::visOdomOptimalTraj(const Trajectory trajectory)
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


  Line.color.r         = 0.0;
  Line.color.g         = 0.0;
  Line.color.b         = 1.0; // 1.0
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
  std::cout << " trajectory optimal size: " << trajectory.trajectory.size() << std::endl;
}


Trajectory localLatticePlanner::chooseBestTraj(vector<Trajectory> &trajectories)
{
  int traj_num = trajectories.size();
  int min_id_num = 0;
  double min_cost = DBL_MAX;
  for(int i = 0; i < traj_num; i++){
    Trajectory traj_cur_ = trajectories[i];
    getTrajScore(traj_cur_);
    if(traj_cur_.cost < min_cost){
      min_cost = traj_cur_.cost;
      min_id_num = i;
    }
  }
  if(min_cost == DBL_MAX) _all_traj_fail = true;
  else _all_traj_fail = false;
  Trajectory output_traj = trajectories[min_id_num];
  return output_traj;
}

double localLatticePlanner::normalizeRad(double val_)
{
  if(val_ >= 0 && val_< 2 * M_PI) return val_;
  else if(val_ >= 2 * M_PI){
    val_ = val_ - 2 * M_PI;
  return normalizeRad(val_);
  }
  else{
    val_ = val_ + 2 * M_PI;
    return normalizeRad(val_);
  }
}

void localLatticePlanner::getTrajScore(Trajectory &trajectory)
{
  int traj_size = trajectory.trajectory.size();
  Vector3d end_pos = trajectory.trajectory[traj_size - 1];
  double density = _map_receiver.getDensity(end_pos[0], end_pos[1]);
  double min_rad_ = 2 * M_PI;
  double max_rad_ = 0;
  for(auto dur_pos: trajectory.trajectory){
    double cur_rad = dur_pos[2];
    double cur_rad_reg = normalizeRad(cur_rad);
    if(cur_rad_reg < min_rad_) min_rad_ = cur_rad_reg ;
    if(cur_rad_reg > max_rad_) max_rad_ = cur_rad_reg ;
    if(_map_receiver.isObs(dur_pos[0], dur_pos[1])) {trajectory.is_collision = true;}
  }
  double err_rad_ = max_rad_ - min_rad_;
  if(err_rad_ <= 0) err_rad_ = 0;
  double target_com = _sur_weight * density ;
  double rad_vary_com = _rad_vary_weight * err_rad_;
  trajectory.cost = target_com + rad_vary_com;
  if(trajectory.is_collision) trajectory.cost = DBL_MAX;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "local_planner");
  const ros::NodeHandle& nh = ros::NodeHandle("~");
  localLatticePlanner local_lattice_planner(nh);
  ros::spin();
  return 0;
}
