#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include "map_generator/ped_xvi.h"
#include "map_generator/quad_tree.h"
#include "ped_navigation/m_ped_xvi.h"

class collisionChecker
{
public:
  collisionChecker();  
  inline Vector2i coord2Idx(double x_, double y_);
  void setStaticMap(vector<int> static_map_);
  void setRobotPos(Vector3d robot_pos_);
  bool isObs(double x_, double y_);
  void setMapSize(Vector2d resolution_, Vector2d offset_, Vector2i map_size_);
  void setPeds(const vector<ped_navigation::m_ped_xvi> ped_coord_list_);
  bool checkCollision();
  bool checkCollision(int & agent_id_, Vector2d & agent_pos);
private:
  // There are two methods to check collision,
  // one is check the static map;
  // the other is to calculate the pedestrians around the robot. Here we use the first
  // but we offer the tube to use the pedestrian information
  quadTree _quad_tree = quadTree();
  Vector3d _robot_pos;
  vector<single_ped_xvi> _cur_peds_xvi;
  vector<int> _static_map;

  Vector2d _resolution;
  Vector2d _inv_resolution;
  Vector2d _offset;
  Vector2i _map_size;

  bool _data_ready_label = false;
  bool _exec_ready_label = true;
  double _dist_safe = 0.6;
  int _collision_mem_id = -1;


};

inline Vector2i collisionChecker::coord2Idx(double x_, double y_)
{
  Vector2i idx;
  idx << min( max( int( (x_ - _offset[0]) * _inv_resolution[0]), 0), _map_size[0] -1),
         min( max( int( (y_ - _offset[1]) * _inv_resolution[1]), 0), _map_size[1] -1);
  return idx;
}

#endif // COLLISION_CHECKER_H
