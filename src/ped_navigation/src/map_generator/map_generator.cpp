#include "map_generator/map_generator.h"

mapGenerator::mapGenerator(ros::NodeHandle nh)
{
  _ped_subscriber = nh.subscribe("/pedsim_simulator/simulated_agents", 1, &mapGenerator::pedestrian_callback, this);
  //_robot_pose_sub = nh.subscribe("/pedsim_simulator/robot_position", 1, &mapGenerator::robot_pose_callback, this);
  _timer = nh.createTimer(ros::Duration(0.1), &mapGenerator::timer_callback, this);
  _map_utils_pub = nh.advertise<ped_navigation::map_utils>("/map_utils",1);
  // get size get lower dim x,y and w, h
  Vector4d ms = _density_map.getSize();
  _quad_tree.setDim(0,ms[0],ms[1],ms[2],ms[3]);
  Vector2i idx_index = _density_map.getMapIdxSize();
  GLX_SIZE = idx_index[0];
  GLY_SIZE = idx_index[1];
  if(draw_density_label){

  }
}

void mapGenerator::pedestrian_callback(const pedsim_msgs::AgentStatesPtr data)
{
  data_ready_ = false;
  cur_peds_xvi.clear();
  for (auto it = data->agent_states.begin(); it != data->agent_states.end(); ++it){
    single_ped_xvi cur_ped_xvi = single_ped_xvi(*it);
    cur_peds_xvi.push_back(cur_ped_xvi);
  }
  //_quad_tree.get_tree_from_list(cur_peds_xvi);
  data_ready_ = true;
}

void mapGenerator::timer_callback(const ros::TimerEvent &)
{
  auto start = system_clock::now();
  updateMap();
  //testUpdateMap();
  _density_map.drawMap();
  _velocity_map.drawMap();
  _cluster_map.drawMap();
  auto end = system_clock::now();
  auto duration = duration_cast<microseconds>(end - start);
  cout<<"it takes"
     << double(duration.count()) * microseconds::period::num / microseconds::period::den
     << "seconds to get lattice planner" << endl;
  publishMap();
}

void mapGenerator::updateMap()
{
  if(!data_ready_)
    return;
  executing_ = true;
  _quad_tree.get_tree_from_list(cur_peds_xvi);
  _density_map.refresh_map();
  _velocity_map.refresh_map();
  _static_map.refresh_map();
  _cluster_map.refresh_map();
  //setObsByPed();
  double thres_dist = 6.25;
  for(int idx_x = 0; idx_x < GLX_SIZE; idx_x++){
    for(int idx_y = 0; idx_y < GLY_SIZE; idx_y++){
      Vector2i pos_base_int(idx_x, idx_y);
      //if(_static_map.isObs(pos_base_int)){continue;}
      if(_density_map.getValue(pos_base_int) == DBL_MAX){continue;}
      Vector2d pos_base = _static_map.gridIndex2coord(pos_base_int);
      double density = 0;
      double s_distance = 10;
      Vector2d velocity(0,0);
      int min_cluster_id_ = -1;
      double min_dist_sq_ = 4;
      vector<single_ped_xvi> near_ped_list;
      _quad_tree.getNeighbors(near_ped_list,pos_base[0],pos_base[1],2.5);
      for(auto ped_pos : near_ped_list){
        double ped_x = ped_pos.agent_pos[0];
        double ped_y = ped_pos.agent_pos[1];

        double distance_square = pow(ped_x - pos_base[0],2)+pow(ped_y - pos_base[1],2);
        if(distance_square <= thres_dist){
          if(distance_square < min_dist_sq_ ){
            min_dist_sq_ = distance_square;
            min_cluster_id_ = ped_pos.cluster_id;
          }
          double partial_density = calcGaussian(distance_square,1);
          density += partial_density;
          velocity[0] += partial_density * ped_pos.agent_vel[0];
          velocity[1] += partial_density * ped_pos.agent_vel[1];
        }
      }
      if(density == 0){_velocity_map.setValue(pos_base_int, velocity);}
      else{velocity[0] /= density;
           velocity[1] /= density;
      _velocity_map.setValue(pos_base_int, velocity);}
      _density_map.setValue(pos_base_int, sqrt(min_dist_sq_));
      //_density_map.setValue(pos_base_int, density);
      if(min_cluster_id_ > 0) _cluster_map.setValue(pos_base_int,min_cluster_id_);
    }
  }
  executing_ = false;
}

void mapGenerator::setObsByPed()
{
  for(auto ped: cur_peds_xvi){
    _static_map.setPedObs(ped.agent_pos);
  }
}

void mapGenerator::testUpdateMap()
{
  if(!data_ready_)
    return;
  executing_ = true;

  _density_map.refresh_map();
  _velocity_map.refresh_map();

  executing_ = false;
}

double mapGenerator::calcGaussian(double x, double h)
{
  double molecule = x;
  double dominator = 2 * h * h; // Here we do not list 2 * h * h
  double left = 1 / (3.14 * dominator);
  return left * exp(- molecule /dominator);
}

ped_navigation::map_utils mapGenerator::preparePubMap()
{
  ped_navigation::m_density_map density_info;
  ped_navigation::m_static_map static_info;
  ped_navigation::m_cluster_map cluster_info;
  ped_navigation::m_velocity_map velocity_info;
  ped_navigation::map_utils map_util_info;
  //  initialize parameters of the map util information
  map_util_info.resolutions.x = _density_map.getResolution();
  map_util_info.resolutions.y = _density_map.getResolution();
  map_util_info.size_x = GLX_SIZE;
  map_util_info.size_y = GLY_SIZE;
  //  get map of static, density and velocity
  density_info.map = _density_map.getMap();
  static_info.map = _static_map.getMap();
  cluster_info.map = _cluster_map.getMap();
  std::vector<Vector2d> vel_map = _velocity_map.getMap();
  std::vector<double> ped_coords_;
  std::vector<ped_navigation::m_ped_xvi> peds_info;
  for(auto ped: cur_peds_xvi){
    ped_navigation::m_ped_xvi ped_info;
    ped_info.id = ped.agent_id;
    ped_info.x = ped.agent_pos[0];
    ped_info.y = ped.agent_pos[1];
    peds_info.push_back(ped_info);
  }
  for(auto idx: vel_map){
    velocity_info.map_x.push_back(idx[0]);
    velocity_info.map_y.push_back(idx[1]);
  }
  //  assign these map to map_util information
  map_util_info.density_map = density_info;
  map_util_info.velocity_map = velocity_info;
  map_util_info.static_map = static_info;
  map_util_info.cluster_map = cluster_info;
  map_util_info.peds_info = peds_info;
  return map_util_info;
}

void mapGenerator::publishMap()
{
  ped_navigation::map_utils map_util_info = preparePubMap();
  _map_utils_pub.publish(map_util_info);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_utils");
  const ros::NodeHandle& nh = ros::NodeHandle("~");
  mapGenerator map_utils(nh);
  ros::spin();
  return 0;
}
