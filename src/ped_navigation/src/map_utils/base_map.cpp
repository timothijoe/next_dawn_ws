#include "map_utils/base_map.h"
template<typename T>
baseMap<T>:: baseMap()
{
  Vector2d map_lower_(0,0);
  Vector2d map_higher_(40,18);
  double resolution_ = 0.3; //0.1
  double inv_res_ = 1.0 / resolution_;
  double max_x_id = (int)(map_higher_(0) * inv_res_);
  double max_y_id = (int)(map_higher_(1) * inv_res_);
  initGridMap(resolution_, map_lower_, map_higher_, max_x_id, max_y_id);
}

template<typename T>
void baseMap<T>:: initGridMap(double resolution_, Vector2d global_xy_l_, Vector2d global_xy_u_, int max_x_id_, int max_y_id_)
{
  gl_xl = global_xy_l_(0);
  gl_yl = global_xy_l_(1);
  gl_xu = global_xy_u_(0);
  gl_yu = global_xy_u_(1);

  GLX_SIZE = max_x_id_;
  GLY_SIZE = max_y_id_;
  GLXY_SIZE = GLX_SIZE * GLY_SIZE;

  resolution = resolution_;
  inv_resolution = 1.0 / resolution_;

  std::vector<T> v1(GLXY_SIZE);
  map = v1;
  zero_map = v1;

}

template<typename T>
void baseMap<T>:: refresh_map()
{
  map = zero_map;
}

template<typename T>
Vector2d baseMap<T>:: gridIndex2coord(const Vector2i &index_)
{
  Vector2d pt;
  pt(0) = ((double)index_(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index_(1) + 0.5) * resolution + gl_xl;
  return pt;
}

template<typename T>
Vector2i baseMap<T>:: coord2gridIndex(const Vector2d &pt_)
{
  Vector2i idx;
  idx << min( max( int( (pt_(0) - gl_xl) * inv_resolution), 0), GLX_SIZE -1),
         min( max( int( (pt_(1) - gl_yl) * inv_resolution), 0), GLY_SIZE -1);
  return idx;
}

template<typename T>
Vector2d baseMap<T>:: coordRounding(const Vector2d &coord_)
{
  return gridIndex2coord(coord2gridIndex(coord_));
}

template<typename T>
Vector4d baseMap<T>:: getSize()
{
  Vector4d map_size(gl_xl, gl_yl, gl_xu - gl_xl, gl_yu - gl_yl);
  return map_size;
}

template<typename T>
Vector2i baseMap<T>:: getMapIdxSize()
{
  Vector2i idxSize(GLX_SIZE,GLY_SIZE);
  return idxSize;
}

template<typename T>
std::vector<T> baseMap<T>::getMap()
{
  return map;
}

template<typename T>
double baseMap<T>:: getResolution()
{
  return resolution;
}


template class baseMap<double>;
template class baseMap<int>;
template class baseMap<Vector2d>;
