#include "map_utils/static_map.h"

staticMap::staticMap()
{
  this->getMapByHand();
  this->zero_map = this->map;
}

void staticMap::getMapByHand()
{
//  Vector4d obst;
//  obst << 0,0,5,5;
//  get_map_from_program(obst);
//  obst << 0,12, 25, 18;
//  get_map_from_program(obst);
//  obst << 30, 12, 40,18;
//  get_map_from_program(obst);
//  obst << 10, 0, 40, 5;
  //  get_map_from_program(obst);
}

//bool staticMap::setMap(vector<int> mapp)
//{
//  map = mapp;
//}

// we set the obstacle when map = 1
// obstacle free if map = 0
bool staticMap::setObs(Vector2i idx_)
{
  return setValue(idx_, 1);
}

bool staticMap::setObs(Vector2d pos_)
{
  return setValue(pos_, 1);
}

// getValue = 1 if it is occluded by obstacle
bool staticMap::isObs(Vector2i idx_)
{
  if (isOutside(idx_)) return true;
  return (getValue(idx_) == 1? true:false);
}

bool staticMap::isObs(Vector2d pos_)
{
  Vector2i idx_ = this->coord2gridIndex(pos_);
  return isObs(idx_);
}

bool staticMap::setPedObs(Vector2d pos_)
{
  Vector2i idx_ = this->coord2gridIndex(pos_);
  for(int i = -2; i <= 2; i++){
    for(int j = -2; j <= 2; j++){
      Vector2i idx_travel(idx_[0] + i, idx_[1] + j);
      setObs(idx_travel);
    }
  }
  return true;
}

void staticMap::get_map_from_program(Vector4d free_space)
{
  Vector2d left_down_pt(free_space[0], free_space[1]);
  Vector2d right_up_pt(free_space[2], free_space[3]);
  Vector2i LEFT_DOWN_INDEX = this->coord2gridIndex(left_down_pt);
  Vector2i RIGHT_UP_INDEX = this->coord2gridIndex(right_up_pt);
  for(int x = LEFT_DOWN_INDEX[0]; x <= RIGHT_UP_INDEX[0]; x++){
    for(int y = LEFT_DOWN_INDEX[1]; y <= RIGHT_UP_INDEX[1]; y++){
      Vector2i proposed_obs;
      proposed_obs << x, y;
      setObs(proposed_obs);
    }
  }
}

cv::Mat staticMap::drawMap( int number, double w_disp_resolution, double h_disp_resolution, bool display)
{
  int w_map_blk_pxl = ceil(resolution / w_disp_resolution);
  int h_map_blk_pxl = ceil(resolution / h_disp_resolution);

  cv::Mat img_gray = cv::Mat::zeros(GLY_SIZE * w_map_blk_pxl, GLX_SIZE * h_map_blk_pxl,CV_8UC1);
  for(int i = 0; i < GLY_SIZE * w_map_blk_pxl; i++){
    for(int j = 0; j < GLX_SIZE * h_map_blk_pxl; j++){
      int map_i = (i - i % w_map_blk_pxl) / w_map_blk_pxl;
      int map_j = (j - j % h_map_blk_pxl) / h_map_blk_pxl;
      if(map[-map_i + (map_j+1) * GLY_SIZE] == 1){
        img_gray.at<uchar>(i,j) = 255;
        continue;
      }
      //if(map[-map_i + (map_j+1) * GLY_SIZE] == 0) continue;
      //  img_gray.at<uchar>(i,j) = (30 * map[-map_i + (map_j+1) * GLY_SIZE]  > 255)? 255: 30 * map[-map_i + (map_j+1) * GLY_SIZE] ;
    }
  }
  cv::Mat dst;
  if(number > 0){
    cv::applyColorMap(img_gray, dst, cv::COLORMAP_JET);
    if (display) {
     std::string title = "Og Map" + std::to_string(number) + "-th";
     cv::imshow(title, dst);
     //cv::imshow("Og Map", dst);
     cv::waitKey(1);
     //cv::destroyWindow("Density Map");
    }

  }
//  cv::applyColorMap(img_gray, dst, cv::COLORMAP_JET);
//  if (display) {
//   cv::imshow("Og Map", dst);
//   cv::waitKey(1);
//   //cv::destroyWindow("Density Map");
//  }
  return dst;


}
