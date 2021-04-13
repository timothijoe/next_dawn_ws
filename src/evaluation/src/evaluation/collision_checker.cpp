#include "evaluation/collision_checker.h"

collisionChecker::collisionChecker()
{
  _resolution << 0.3, 0.3;
  _inv_resolution << 1 / _resolution[0], 1 / _resolution[1];
  _offset << 0, 0;
  _map_size << 0, 0;
}

void collisionChecker::setStaticMap(vector<int> static_map_)
{
  _static_map = static_map_;
}

void collisionChecker::setRobotPos(Vector3d robot_pos_)
{
  _robot_pos = robot_pos_;
}

bool collisionChecker::isObs(double x_, double y_)
{
  Vector2i idx_ = coord2Idx(x_, y_);
  return (_static_map[idx_[0] * _map_size[1] + idx_[1]] == 1? true : false);
}

void collisionChecker::setMapSize(Vector2d resolution_, Vector2d offset_, Vector2i map_size_)
{
  _resolution = resolution_;
  _inv_resolution << 1 / _resolution[0], 1 / _resolution[1];
  _offset = offset_;
  _map_size = map_size_;
  _quad_tree.setDim(0,offset_[0],offset_[1],_map_size[0] * _inv_resolution[0],_map_size[1] * _inv_resolution[1]);
}

void collisionChecker::setPeds(const vector<ped_navigation::m_ped_xvi> ped_coord_list_)
{
  _cur_peds_xvi.clear();
  for(auto ped_info: ped_coord_list_){
    single_ped_xvi m = single_ped_xvi(ped_info.id, ped_info.x, ped_info.y);
    _cur_peds_xvi.push_back(m);
  }
}

// if collision happens, return true
bool collisionChecker::checkCollision()
{
  if(isObs(_robot_pos[0], _robot_pos[1]))
    return true;
  if(_cur_peds_xvi.empty())
    return false;
  _quad_tree.get_tree_from_list(_cur_peds_xvi);
  vector<single_ped_xvi> near_ped_list;
  _quad_tree.getNeighbors(near_ped_list, _robot_pos[0], _robot_pos[1], _dist_safe);
  if(!near_ped_list.empty()){
  for(auto ped : near_ped_list){
    Vector2d rp(_robot_pos[0],_robot_pos[1]);
    Vector2d diff = ped.agent_pos - rp;
    if(diff.norm() < _dist_safe){
      _collision_mem_id = ped.agent_id;
      std::cout << "collision happens" << std::endl;
      return true;}
  }}
  //std::cout << "collision_free" << std::endl;
  return false;
}

bool collisionChecker::checkCollision(int &agent_id_, Vector2d &agent_pos)
{
  agent_id_ = 0;
  agent_pos << 0, 0;
  if(isObs(_robot_pos[0], _robot_pos[1]))
    return true;
  if(_cur_peds_xvi.empty())
    return false;
  _quad_tree.get_tree_from_list(_cur_peds_xvi);
  vector<single_ped_xvi> near_ped_list;
  _quad_tree.getNeighbors(near_ped_list, _robot_pos[0], _robot_pos[1], _dist_safe * 2);
  if(!near_ped_list.empty()){
  for(auto ped : near_ped_list){
    Vector2d rp(_robot_pos[0],_robot_pos[1]);
    Vector2d diff = ped.agent_pos - rp;
    //std::cout << "dist is: " << diff.norm() << std::endl;
    if(diff.norm() < _dist_safe){
      _collision_mem_id = ped.agent_id;
      agent_id_ = ped.agent_id;
      agent_pos << ped.agent_pos[0], ped.agent_pos[1];
      std::cout << "collision happens" << std::endl;
      return true;}
  }}
  //std::cout << "collision_free" << std::endl;
  return false;
}
