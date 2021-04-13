#include "map_utils/velocity_map.h"

velocityMap::velocityMap()
{
  getMapByHand();
  zero_map = map;
}

void velocityMap::getMapByHand()
{
//  Vector4d obst;
//  obst << 0,0,5,5;
//  setUnReach(obst);
//  obst << 0,12, 25, 17;
//  setUnReach(obst);
//  obst << 30, 12, 40,17;
//  setUnReach(obst);
//  obst << 10, 0, 40, 5;
//  setUnReach(obst);
}

bool velocityMap::setUnTouch(Vector2i idx_)
{
  Vector2d unTouch(DBL_MAX,DBL_MAX);
  return setValue(idx_, unTouch);
}

bool velocityMap::setUnTouch(Vector2d pos_)
{
  Vector2d unTouch(DBL_MAX,DBL_MAX);
  return setValue(pos_,unTouch);
}

bool velocityMap::setUnReach(Vector4d unreach_space_)
{
  Vector2d left_down_pt(unreach_space_[0], unreach_space_[1]);
  Vector2d right_up_pt(unreach_space_[2], unreach_space_[3]);
  Vector2i LEFT_DOWN_INDEX = this->coord2gridIndex(left_down_pt);
  Vector2i RIGHT_UP_INDEX = this->coord2gridIndex(right_up_pt);
  for(int x = LEFT_DOWN_INDEX[0]; x <= RIGHT_UP_INDEX[0]; x++){
    for(int y = LEFT_DOWN_INDEX[1]; y <= RIGHT_UP_INDEX[1]; y++){
      Vector2i proposed_obs;
      proposed_obs << x, y;
      setUnTouch(proposed_obs);
    }
  }
}

cv::Mat velocityMap::drawMap(double w_disp_resolution, double h_disp_resolution, bool display)
{
  int w_map_blk_pxl = ceil(resolution / w_disp_resolution);
  int h_map_blk_pxl = ceil(resolution / h_disp_resolution);

  cv::Mat velocity_map(GLY_SIZE * w_map_blk_pxl, GLX_SIZE * h_map_blk_pxl,CV_8UC3,cv::Scalar(255, 255, 255));
  for(int i = 0; i < GLY_SIZE; i++){
    for(int j = 0; j < GLX_SIZE; j++){
      //int map_i = (i - i % w_map_blk_pxl) / w_map_blk_pxl;
      //int map_j = (j - j % h_map_blk_pxl) / h_map_blk_pxl;
      int map_i = i;
      int map_j = j;
      if(map[-map_i + (map_j+1) * GLY_SIZE][0] == 0 && map[-map_i + (map_j+1) * GLY_SIZE][1] == 0)
      {continue;}
      if(map[-map_i + (map_j+1) * GLY_SIZE][0] == DBL_MAX || map[-map_i + (map_j+1) * GLY_SIZE][1] == DBL_MAX)
      {continue;}
      int pt_1_x = i * w_map_blk_pxl + w_map_blk_pxl / 2;
      int pt_1_y = j * w_map_blk_pxl + w_map_blk_pxl / 2 ;
      double x = map[-map_i + (map_j+1) * GLY_SIZE][0];
      double y = map[-map_i + (map_j+1) * GLY_SIZE][1];
      //double scale = sqrt(x^2 + y^2);
      double dx = y / (double)w_disp_resolution;
      double dy = x / (double)h_disp_resolution;

      cv::arrowedLine(velocity_map, \
              cv::Point(pt_1_y, pt_1_x), \
              cv::Point((double)pt_1_y + dy, (double)pt_1_x + dx), \
              cv::Scalar(0, 0, 255), \
              1, 8, 0, 0.1);
    }
  }
  if (display) {
      //cv::namedWindow("Flow Field", cv::WINDOW_NORMAL);
      cv::imshow("Flow Field", velocity_map);
      cv::waitKey(1);
  }
  return velocity_map;
}
