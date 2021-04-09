#ifndef FLOWSEGMENTS_H
#define FLOWSEGMENTS_H

#include "math.h"
#include <Eigen/Eigen>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ctime>
#include <chrono>

using namespace std;
using namespace Eigen;

class FlowSegment{
public:
  FlowSegment(){};
  FlowSegment(int idx, double s_x, double s_y, double e_x, double e_y){
    id = idx;
    start_point << s_x, s_y;
    end_point << e_x, e_y;
  }
  inline Vector2d min_point_with_outpoint(Vector2d point)
  {
    Eigen::Vector2d diff = end_point - start_point;
    double sq_norm = diff.squaredNorm();
    if (sq_norm == 0)
      return start_point;
    double u = ((point.x() - start_point.x()) * diff.x() + (point.y() - start_point.y())*diff.y()) / sq_norm;
    if (u <= 0) return start_point;
    else if (u >= 1) return end_point;
    return start_point + u*diff;
  }

  inline double min_dist_with_point(Vector2d point){
    Vector2d min_dist_point = min_point_with_outpoint(point);
    return (point - min_dist_point).squaredNorm();
  }
  int id;
  Vector2d start_point;
  Vector2d end_point;
  double width = 1.0;

};
class FlowSegments
{
public:
  FlowSegments();
  int id;
  vector<FlowSegment> segments;
  int cur_seg_id = -1;
  FlowSegment target_seg;

  int find_nearest_segment(Vector2d point);
  int find_nearest_segment_with_dist(Vector2d point, double& dist);
  bool find_nearest_segment_and_dist(Vector2d point, FlowSegment& seg,double& dist);


};

#endif // FLOWSEGMENTS_H
