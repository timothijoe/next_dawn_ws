#ifndef BASE_MAP_H
#define BASE_MAP_H

#include "math.h"
#include <Eigen/Eigen>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ctime>
#include <chrono>


#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace std;
using namespace Eigen;

// if obstacle map, T = uint8;
// if density map, T = double;
// if velocity map, T = Vector2d;
template <typename T>
class baseMap
{
public:
  baseMap();
  void initGridMap(double resolution_, Vector2d global_xy_l_, Vector2d global_xy_u_, int max_x_id_, int max_y_id_);
  //virtual void getMapByHand();
  void refresh_map();
  Vector2d gridIndex2coord(const Vector2i & index_);
  Vector2i coord2gridIndex(const Vector2d & pt_);
  Vector2d coordRounding(const Vector2d & coord_);
  inline bool isOutside(Vector2i index_);
  inline T getValue(Vector2d pos_);
  inline T getValue(Vector2i pos_id_);
  inline bool setValue(Vector2d position_, T value_);
  inline bool setValue(Vector2i pos_id, T value_);
//  virtual cv::Mat drawMap(double w_disp_resolution, double h_disp_resolution, bool display=false);
  Vector4d getSize();
  Vector2i getMapIdxSize();
  std::vector<T> getMap();
  double getResolution();

protected:
  // map attibute information
  int GLX_SIZE, GLY_SIZE;
  int GLXY_SIZE;
  double resolution, inv_resolution;
  double gl_xl, gl_yl;
  double gl_xu, gl_yu;
  // map itself
  std::vector<T> map;
  std::vector<T> zero_map;
};

template<typename T>
inline bool baseMap<T>::isOutside(Vector2i index_)
{
    int one_line_idx = index_(1) + index_(0) * GLY_SIZE;
    if(one_line_idx < 0 || one_line_idx >= GLXY_SIZE){
      return true;
    }
    return false;
}

template<typename T>
inline T baseMap<T>::getValue(Vector2d pos_)
{
  Vector2i idx = coord2gridIndex(pos_);
  return map[idx[0] * GLY_SIZE + idx[1]];
}

template<typename T>
inline T baseMap<T>::getValue(Vector2i idx_)
{
  return map[idx_[0] * GLY_SIZE + idx_[1]];
}

template<typename T>
inline bool baseMap<T>::setValue(Vector2d position_, T value_)
{
  Vector2i target_idx = coord2gridIndex(position_);
  return setValue(target_idx, value_);
}

template<typename T>
inline bool baseMap<T>::setValue(Vector2i pos_id, T value_)
{
  if(isOutside(pos_id)){
    return false;
  }
  else{
    map[ pos_id[0]*GLY_SIZE + pos_id[1] ] = value_;
    return true;
  }
}

#endif // BASE_MAP_H
