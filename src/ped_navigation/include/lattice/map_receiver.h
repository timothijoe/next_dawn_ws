#ifndef MAP_RECEIVER_H
#define MAP_RECEIVER_H

#include "math.h"
#include <Eigen/Eigen>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ctime>
#include <chrono>

using namespace std;
using namespace Eigen;

class mapReceiver
{
public:
  mapReceiver();
  inline Vector2i   coord2Idx(double x_, double y_);
  bool     isObs(double x_, double y_);
  bool     isOutSide(double x_, double y_);
  int      getCluster(double x_, double y_);
  double   getDensity(double x_, double y_);
  Vector2d getVelocity(double x_, double y_);
  void     setStaticMap(vector<int> static_map_);
  void     setClusterMap(vector<int> cluster_map_);
  void     setDensityMap(vector<double> density_map_);
  void     clearOgMaps();
  void     addOgMap(vector<int> static_map_);
  bool     checkOgCollision(int ix_, int iy_, int m);

  void     setVelMap(vector<double> vel_map_x_, vector<double> vel_map_y_);
  void     setMapSize(Vector2d resolution_, Vector2d offset_, Vector2i map_size_);

  Vector2d _resolution;
  Vector2d _inv_resolution;
  Vector2d _offset;
  Vector2i _map_size;

  vector<int> _static_map;
  vector<int> _cluster_map;
  vector<double> _density_map;
  vector<double> _vel_x_map;
  vector<double> _vel_y_map;
  vector<vector<int>> _og_maps;

  bool _map_inited = false;

};

inline Vector2i mapReceiver::coord2Idx(double x_, double y_)
{
  Vector2i idx;
  idx << min( max( int( (x_ - _offset[0]) * _inv_resolution[0]), 0), _map_size[0] -1),
         min( max( int( (y_ - _offset[1]) * _inv_resolution[1]), 0), _map_size[1] -1);
  return idx;
}

#endif // MAP_RECEIVER_H
