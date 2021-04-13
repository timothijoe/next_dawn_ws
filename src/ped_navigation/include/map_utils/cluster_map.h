#ifndef CLUSTER_MAP_H
#define CLUSTER_MAP_H

#include "map_utils/base_map.h"
#include <limits>
class clusterMap : public baseMap<int>
{
public:
  clusterMap();

  void getMapByHand();
  bool setUnTouch(Vector2i idx_);
  bool setUnTouch(Vector2d pos_);
  bool setUnReach(Vector4d unreach_space_);
  cv::Mat drawMap(double w_disp_resolution = 0.05, double h_disp_resolution = 0.05, bool display=true);
};

#endif // CLUSTER_MAP_H
