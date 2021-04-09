#include "flowsegments.h"
#include "math.h"
FlowSegments::FlowSegments()
{
  segments.clear();
  segments.push_back(FlowSegment(0, 0, 0, 0.6, 3.42));
  segments.push_back(FlowSegment(1, 0.6, 3.42, 5.0, 8.66));
  segments.push_back(FlowSegment(2, 5.0, 8.66, 8.26, 9.8));
  segments.push_back(FlowSegment(3, 8.26, 9.8, 11.74, 10));
  segments.push_back(FlowSegment(4, 11.74, 10, 15, 8.66));
  segments.push_back(FlowSegment(5, 15, 8.66, 17.66, 6.42));
  segments.push_back(FlowSegment(6, 17.66, 6.42, 19.4, 3.42));
  segments.push_back(FlowSegment(7, 19.4, 3.42, 20, 0));
}


int FlowSegments::find_nearest_segment(Vector2d point)
{
  double nearest_dist = 100;
  int nearest_id = -1;
  for(auto seg: segments){
    double dist = seg.min_dist_with_point(point);
    if(dist < nearest_dist){
      nearest_dist = dist;
      nearest_id = seg.id;
    }
  }
  return nearest_id;
}

int FlowSegments::find_nearest_segment_with_dist(Vector2d point, double& dist)
{
  double nearest_dist = 10;
  int nearest_id = -1;
  for(auto seg: segments){
    double dist = seg.min_dist_with_point(point);
    if(dist < nearest_dist){
      nearest_dist = dist;
      nearest_id = seg.id;
    }
  }
  dist = nearest_dist;
  return nearest_id;

}

bool FlowSegments::find_nearest_segment_and_dist(Vector2d point, FlowSegment &seg_, double &dist)
{
  double nearest_dist = 1000;
  int nearest_id = -1;
  for(auto seg: segments){
    double dist = seg.min_dist_with_point(point);
    if(dist < nearest_dist){
      nearest_dist = dist;
      nearest_id = seg.id;
    }
  }
  dist = nearest_dist;
  seg_ = segments[nearest_id];
}
