#include "map_utils/cluster_map.h"

clusterMap::clusterMap()
{
  getMapByHand();
  zero_map = map;
}

void clusterMap::getMapByHand()
{

//    Vector4d obst;
//    obst << 0,0,5,5;
//    setUnReach(obst);
//    obst << 0,12, 25, 18;
//    setUnReach(obst);
//    obst << 30, 12, 40,18;
//    setUnReach(obst);
//    obst << 10, 0, 40, 5;
//    setUnReach(obst);

}

bool clusterMap::setUnTouch(Vector2i idx_)
{
  return setValue(idx_, INT_MAX);
}

bool clusterMap::setUnTouch(Vector2d pos_)
{
  return this->setValue(pos_, INT_MAX);
}

bool clusterMap::setUnReach(Vector4d unreach_space_)
{
  Vector2d left_down_pt(unreach_space_[0], unreach_space_[1]);
  Vector2d right_up_pt(unreach_space_[2], unreach_space_[3]);
  Vector2i LEFT_DOWN_INDEX = coord2gridIndex(left_down_pt);
  Vector2i RIGHT_UP_INDEX = coord2gridIndex(right_up_pt);
  for(int x = LEFT_DOWN_INDEX[0]; x <= RIGHT_UP_INDEX[0]; x++){
    for(int y = LEFT_DOWN_INDEX[1]; y <= RIGHT_UP_INDEX[1]; y++){
      Vector2i proposed_obs;
      proposed_obs << x, y;
      setUnTouch(proposed_obs);
    }
  }
}

cv::Mat clusterMap::drawMap(double w_disp_resolution, double h_disp_resolution, bool display)
{
  int w_map_blk_pxl = ceil(resolution / w_disp_resolution);
  int h_map_blk_pxl = ceil(resolution / h_disp_resolution);

  cv::Mat img_gray = cv::Mat::zeros(GLY_SIZE * w_map_blk_pxl, GLX_SIZE * h_map_blk_pxl,CV_8UC1);
  for(int i = 0; i < GLY_SIZE * w_map_blk_pxl; i++){
    for(int j = 0; j < GLX_SIZE * h_map_blk_pxl; j++){
      int map_i = (i - i % w_map_blk_pxl) / w_map_blk_pxl;
      int map_j = (j - j % h_map_blk_pxl) / h_map_blk_pxl;
      if(map[-map_i + (map_j+1) * GLY_SIZE] == INT_MAX){
        img_gray.at<uchar>(i,j) = 255;
        continue;
      }
      if(map[-map_i + (map_j+1) * GLY_SIZE] == 0) continue;
        img_gray.at<uchar>(i,j) = (30 * map[-map_i + (map_j+1) * GLY_SIZE]  > 255)? 255: 30 * map[-map_i + (map_j+1) * GLY_SIZE] ;
    }
  }
  cv::Mat dst;
  cv::applyColorMap(img_gray, dst, cv::COLORMAP_JET);
  if (display) {
   cv::imshow("Cluster Map", dst);
   cv::waitKey(1);
   //cv::destroyWindow("Density Map");
  }
  return dst;
}

