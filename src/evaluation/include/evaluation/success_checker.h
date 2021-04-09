#ifndef SUCCESS_CHECKER_H
#define SUCCESS_CHECKER_H

#include <iostream>
#include <ratio>
#include <chrono>
#include <Eigen/Eigen>

typedef std::chrono::time_point<std::chrono::system_clock> TIME_POINT;
using namespace std;
using namespace Eigen;
using namespace std::chrono;
class successChecker
{
public:
enum metaTask {
    INTO_FLOW,
    OUT_OF_FLOW,
    CROSS_FLOW,
    ALONG_FLOW,
    Backward_FLOW,
    NORMAL
};
  successChecker();
  bool setMetaTask(metaTask task_type_ = INTO_FLOW);
  bool setFlowToInteract(int cluster_id_);
  bool isSuccess();
  bool isIntoFlowSucc();
  bool isOutOfFlowSucc();
  bool isCrossFlowSucc();
  bool isAlongFlowSucc();
  bool isBackWardFlowSucc();
  bool isNormalSucc();

  int      getCluster(double x_, double y_);
  Vector2d getVelocity(double x_, double y_);

  // receiving map
  inline Vector2i coord2Idx(double x_, double y_);
  void setClusterMap(vector<int> cluster_map_);
  void setVelMap(vector<double> vel_map_x_, vector<double> vel_map_y_);
  void setRobotPos(Vector3d robot_pos_);
  void setMapSize(Vector2d resolution_, Vector2d offset_, Vector2i map_size_);
  bool checkMatchCluster();

private:
  // meta task parameters
  metaTask _task_type;
  int _flow_to_interact = 1;
  bool _instant_success;
  Vector3d _robot_pos;
  double _succ_time_length = 3; // we must guarantee the robot keep pace of 3 seconds
  TIME_POINT _start_succ_time;

  // map classification
  vector<int> _cluster_map;
  vector<double> _vel_x_map;
  vector<double> _vel_y_map;

  // map parameters
  Vector2d _resolution;
  Vector2d _inv_resolution;
  Vector2d _offset;
  Vector2i _map_size;

};

Vector2i successChecker::coord2Idx(double x_, double y_)
{
  Vector2i idx;
  idx << min( max( int( (x_ - _offset[0]) * _inv_resolution[0]), 0), _map_size[0] -1),
         min( max( int( (y_ - _offset[1]) * _inv_resolution[1]), 0), _map_size[1] -1);
  return idx;
}

#endif // SUCCESS_CHECKER_H
