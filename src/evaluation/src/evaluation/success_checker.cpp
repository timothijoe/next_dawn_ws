#include "evaluation/success_checker.h"

successChecker::successChecker()
{
  _resolution << 0.3, 0.3;
  _inv_resolution << 1 / _resolution[0], 1 / _resolution[1];
  _offset << 0, 0;
  _map_size << 0, 0;
  setMetaTask();

}

bool successChecker::setMetaTask(successChecker::metaTask meta_task_)
{
  _task_type = meta_task_;
}

bool successChecker::setFlowToInteract(int cluster_id_)
{
  _flow_to_interact = cluster_id_;
}


bool successChecker::isSuccess()
{
  bool success_label;
  switch(_task_type){
  case INTO_FLOW:
    success_label = isIntoFlowSucc();
    break;
  case OUT_OF_FLOW:
    success_label = isOutOfFlowSucc();
    break;
  case CROSS_FLOW:
    success_label = isCrossFlowSucc();
    break;
  case ALONG_FLOW:
    success_label = isAlongFlowSucc();
    break;
  case Backward_FLOW:
    success_label = isBackWardFlowSucc();
    break;
  default:
    success_label = isNormalSucc();
  }
  return success_label;

}

bool successChecker::isIntoFlowSucc()
{
  if(!checkMatchCluster()){
    _start_succ_time = system_clock::now();
    return false;
  }else{
    auto now_succ_time =  system_clock::now();
    auto duration = duration_cast<microseconds>(now_succ_time - _start_succ_time);
    double duration_std = double(duration.count()) * microseconds::period::num / microseconds::period::den;
    std::cout << "duration std is : " << duration_std << std::endl;
    if(duration_std <= _succ_time_length)
    {
      return false;
    }
    return true;
    std::cout << "Into Flow success !" << std::endl;
  }
  //return true;
}

bool successChecker::isOutOfFlowSucc()
{
  return false;
}

bool successChecker::isCrossFlowSucc()
{
  return false;
}

bool successChecker::isAlongFlowSucc()
{
  return false;
}

bool successChecker::isBackWardFlowSucc()
{
  return false;
}

bool successChecker::isNormalSucc()
{
  return false;
}

int successChecker::getCluster(double x_, double y_)
{
  Vector2i idx_ = coord2Idx(x_, y_);
  return _cluster_map[idx_[0] * _map_size[1] + idx_[1]];

}

Vector2d successChecker::getVelocity(double x_, double y_)
{
  Vector2d vel_;
  Vector2i idx_ = coord2Idx(x_, y_);
  vel_[0] = _vel_x_map[idx_[0] * _map_size[1] + idx_[1]];
  vel_[1] = _vel_y_map[idx_[0] * _map_size[1] + idx_[1]];
  return vel_;
}

void successChecker::setClusterMap(vector<int> cluster_map_)
{
  _cluster_map = cluster_map_;

}

void successChecker::setVelMap(vector<double> vel_map_x_, vector<double> vel_map_y_)
{
  _vel_x_map = vel_map_x_;
  _vel_y_map = vel_map_y_;

}

void successChecker::setRobotPos(Vector3d robot_pos_)
{
  _robot_pos = robot_pos_;
}

void successChecker::setMapSize(Vector2d resolution_, Vector2d offset_, Vector2i map_size_)
{
  _resolution = resolution_;
  _inv_resolution << 1 / _resolution[0], 1 / _resolution[1];
  _offset = offset_;
  _map_size = map_size_;
}

bool successChecker::checkMatchCluster()
{
  int cluster_id = getCluster(_robot_pos[0], _robot_pos[1]);
  if(cluster_id != _flow_to_interact){
    return false;
  }else{
    Vector2d vel = getVelocity(_robot_pos[0], _robot_pos[1]);
    Vector2d robot_dir = Vector2d(cos(_robot_pos[2]), sin(_robot_pos[2]));
    double vel_correlation = (vel[0] * robot_dir[0] + vel[1] * robot_dir[1]) / vel.norm();
    if(vel_correlation <= 0.8) return false;
    std::cout << "Matcher corrolation: " << vel_correlation << std::endl;
  }
  return true;
}
