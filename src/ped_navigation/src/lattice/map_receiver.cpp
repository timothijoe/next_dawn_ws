#include "lattice/map_receiver.h"

mapReceiver::mapReceiver()
{
  _resolution << 0.3, 0.3;
  _inv_resolution << 1 / _resolution[0], 1 / _resolution[1];
  _offset << 0, 0;
  _map_size << 0, 0;
}

bool mapReceiver::isObs(double x_, double y_)
{
  Vector2i idx_ = coord2Idx(x_, y_);
  return (_static_map[idx_[0] * _map_size[1] + idx_[1]] == 1? true : false);
}

bool mapReceiver::isOutSide(double x_, double y_)
{
  Vector2i idx_ = coord2Idx(x_, y_);
  int one_line_idx = idx_[0] * _map_size[1] + idx_[1];
  if( one_line_idx < 0 || one_line_idx >= _map_size[0] * _map_size[1]){
    return true;}
  return false;
}

int mapReceiver::getCluster(double x_, double y_)
{
  Vector2i idx_ = coord2Idx(x_, y_);
  return _cluster_map[idx_[0] * _map_size[1] + idx_[1]];
}


double mapReceiver::getDensity(double x_, double y_)
{
  Vector2i idx_ = coord2Idx(x_, y_);
  return _density_map[idx_[0] * _map_size[1] + idx_[1]];
}

Vector2d mapReceiver::getVelocity(double x_, double y_)
{
  Vector2d vel_;
  Vector2i idx_ = coord2Idx(x_, y_);
  vel_[0] = _vel_x_map[idx_[0] * _map_size[1] + idx_[1]];
  vel_[1] = _vel_y_map[idx_[0] * _map_size[1] + idx_[1]];
  return vel_;
}

void mapReceiver::setStaticMap(vector<int> static_map_)
{
  _static_map = static_map_;
}

void mapReceiver::setClusterMap(vector<int> cluster_map_)
{
  _cluster_map = cluster_map_;
}

void mapReceiver::setDensityMap(vector<double> density_map_)
{
  _density_map = density_map_;
}

void mapReceiver::clearOgMaps()
{
  _og_maps.clear();
}

void mapReceiver::addOgMap(vector<int> static_map_)
{
  vector<int> og_map_;
  og_map_ = static_map_;
  _og_maps.push_back(og_map_);
}

bool mapReceiver::checkOgCollision(int ix_, int iy_, int m)
{
  vector<int> mp = _og_maps[m];
  return (mp[ix_ * _map_size[1] + iy_] == 1? true : false);

}

void mapReceiver::setVelMap(vector<double> vel_map_x_, vector<double> vel_map_y_)
{
  _vel_x_map = vel_map_x_;
  _vel_y_map = vel_map_y_;
}

void mapReceiver::setMapSize(Vector2d resolution_, Vector2d offset_, Vector2i map_size_)
{
  _resolution = resolution_;
  _inv_resolution << 1 / _resolution[0], 1 / _resolution[1];
  _offset = offset_;
  _map_size = map_size_;
}
