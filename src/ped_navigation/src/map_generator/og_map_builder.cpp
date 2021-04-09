#include "map_generator/og_map_builder.h"

ogMapBuilder::ogMapBuilder(ros::NodeHandle nh)
{
  _ped_subscriber = nh.subscribe("/pedsim_simulator/simulated_agents", 1, &ogMapBuilder::pedestrian_callback, this);
  _timer = nh.createTimer(ros::Duration(0.1), &ogMapBuilder::timer_callback, this);
  _map_utils_pub = nh.advertise<ped_navigation::map_utils>("/map_utils",1);
  // get size get lower dim x,y and w, h
  Vector4d ms = _density_map.getSize();
  _quad_tree.setDim(0,ms[0],ms[1],ms[2],ms[3]);
  Vector2i idx_index = _density_map.getMapIdxSize();
  GLX_SIZE = idx_index[0];
  GLY_SIZE = idx_index[1];

}

void ogMapBuilder::pedestrian_callback(const pedsim_msgs::AgentStatesPtr data)
{
  if(executing_) return;
   data_ready_ = false;
   cur_peds_xvi.clear();
   for (auto it = data->agent_states.begin(); it != data->agent_states.end(); ++it){
     single_ped_xvi cur_ped_xvi = single_ped_xvi(*it);
     cur_peds_xvi.push_back(cur_ped_xvi);
   }
   std::cout << "get pedestrians number: "<< cur_peds_xvi.size()  << std::endl;
   //_quad_tree.get_tree_from_list(cur_peds_xvi);
   data_ready_ = true;
}

void ogMapBuilder::timer_callback(const ros::TimerEvent &)
{
  auto start = system_clock::now();
    updateMap();
    //testUpdateMap();
//    _density_map.drawMap();
    //std::cout << "pedestrian map size: " << vec_static_maps.size() << std::endl;
    //_static_map.setMap(vec_static_maps[9].getMap());
    //_static_map.drawMap();
    vec_static_maps[1].drawMap(1);
    vec_static_maps[9].drawMap(9);
//    _velocity_map.drawMap();
//    _cluster_map.drawMap();
//    for(auto static_map:vec_static_maps){
//      static_map.drawMap();
//    }
//    vec_static_maps[9].drawMap();
    auto end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    cout<<"it takes"
       << double(duration.count()) * microseconds::period::num / microseconds::period::den
       << "seconds to get lattice planner" << endl;
    publishMap();

}

void ogMapBuilder::updateMap()
{
  std::cout << "map updated: " << std::endl;
  if(!data_ready_)
      return;
    executing_ = true;
    _quad_tree.get_tree_from_list(cur_peds_xvi);
    _density_map.refresh_map();
    _velocity_map.refresh_map();
    _static_map.refresh_map();
    _cluster_map.refresh_map();
    vec_static_maps.clear();
    for(int k= 0; k < nbMap; k++){
      vec_static_maps.push_back(_static_map);
    }
    std::cout << "map updated11: " << std::endl;
    for(auto ped: cur_peds_xvi){
      Vector2d temp_pose;
      //std::cout << "pedestrian id: " << ped.agent_id << std::endl;
      int max_i, max_j, min_i, min_j;
      double ped_px = ped.agent_pos[0];
      double ped_py = ped.agent_pos[1];
      double ped_vx = ped.agent_vel[0];
      double ped_vy = ped.agent_vel[1];
      for(int k = 0; k < nbMap; k++){
        temp_pose[0] = ped_px + k * timeStep * ped_vx;
        temp_pose[1] = ped_py + k * timeStep * ped_vy;
        //Vector2i ped_grid_2d = _static_map.coord2gridIndex(temp_pose);
//        min_i = max(ped_grid_2d[0] - 3, 0);
//        max_i = min(ped_grid_2d[0] + 3, GLX_SIZE);
//        min_j = max(ped_grid_2d[1] - 3, 0);
//        max_j = min(ped_grid_2d[1] + 3, GLY_SIZE);
//        for(int i = min_i; i <= max_i; i++){
//          for(int j = min_j; i <= max_j; j++){
//            Vector2i near_ped_area(i,j);
//            vec_static_maps[k].setObs(near_ped_area);
//          }
//        } // end set map

        vec_static_maps[k].setPedObs(temp_pose);
      }// end set nbMap
      //std::cout << "pedestrian id : " << ped.agent_id << std::endl;
    }// end each pedestrian
    executing_ = false;

}

ped_navigation::map_utils ogMapBuilder::preparePubMap()
{
    ped_navigation::m_density_map density_info;
    ped_navigation::m_static_map static_info;
    ped_navigation::m_static_map static_info_iter;
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
    int ii = 0;
    for(auto static_mapp:vec_static_maps){
      static_info_iter.map = vec_static_maps[ii].getMap();
      ii+= 1;
      map_util_info.vec_static_maps.push_back(static_info_iter);
    }
    std::cout << "ii = : " << ii << std::endl;
    //  assign these map to map_util information
    map_util_info.density_map = density_info;
    map_util_info.velocity_map = velocity_info;
    map_util_info.static_map = static_info;
    map_util_info.cluster_map = cluster_info;
    map_util_info.peds_info = peds_info;
    return map_util_info;
}

void ogMapBuilder::publishMap()
{
  ped_navigation::map_utils map_util_info = preparePubMap();
  _map_utils_pub.publish(map_util_info);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_utilss");
  const ros::NodeHandle& nh = ros::NodeHandle("~");
  ogMapBuilder map_utils(nh);
  ros::spin();
  return 0;
}


