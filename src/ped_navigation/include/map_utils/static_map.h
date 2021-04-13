#ifndef STATIC_MAP_H
#define STATIC_MAP_H
#include "map_utils/base_map.h"

class staticMap : public baseMap<int>
{
public:
  using baseMap<int>::getSize;
  staticMap();

  void getMapByHand();
  //bool setMap(vector<int> mapp);
  bool setObs(Vector2i idx_);
  bool setObs(Vector2d pos_);
  bool isObs(Vector2i idx_);
  bool isObs(Vector2d pos_);
  bool setPedObs(Vector2d pos_);
  void get_map_from_program(Vector4d free_space);
  cv::Mat drawMap(int number=0, double w_disp_resolution = 0.05, double h_disp_resolution = 0.05,bool display=true);
};

#endif // STATIC_MAP_H
