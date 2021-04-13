#ifndef DENSITY_MAP_H
#define DENSITY_MAP_H

#include "map_utils/base_map.h"
#include <float.h>
class densityMap : public baseMap<double>
{
public:
  //using baseMap<double>::baseMap;
  densityMap();
  void getMapByHand();
  bool setUnTouch(Vector2i idx_);
  bool setUnTouch(Vector2d pos_);
  bool setUnReach(Vector4d unreach_space_);
  cv::Mat drawMap(double w_disp_resolution = 0.05, double h_disp_resolution = 0.05, bool display=true);
};

#endif // DENSITY_MAP_H
