#ifndef VELOCITY_MAP_H
#define VELOCITY_MAP_H
#include "map_utils/base_map.h"
#include <float.h>
using namespace std;
using namespace Eigen;
//using namespace baseMap<Vector2d>::baseMap;
class velocityMap : public baseMap<Vector2d>
{
public:
  velocityMap();
  void getMapByHand();
  bool setUnTouch(Vector2i idx_);
  bool setUnTouch(Vector2d pos_);
  bool setUnReach(Vector4d unreach_space_);
  cv::Mat drawMap(double w_disp_resolution = 0.05, double h_disp_resolution = 0.05, bool display=true);
};


#endif // VELOCITY_MAP_H
