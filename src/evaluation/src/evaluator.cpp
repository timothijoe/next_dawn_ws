#include "evaluation/evaluator.h"

evaluator::evaluator(ros::NodeHandle nh)
{
  _map_util_sub = nh.subscribe("/map_utils", 1, &evaluator::update_map_util, this);
  _robot_pose_sub = nh.subscribe("/pedsim_simulator/robot_position", 1, &evaluator::robot_pose_callback, this);
  _timer = nh.createTimer(ros::Duration(0.1), &evaluator::timer_callback, this);
  _evaluation_pub = nh.advertise<evaluation::evaluation_sending>("/evaluation_information",1);
}

void evaluator::update_map_util(const ped_navigation::map_utilsPtr data)
{
  data_ready = false;
  Vector2d origin_( data->origin.x, data->origin.y);
  Vector2d resolution_(data->resolutions.x, data->resolutions.y);
  Vector2i map_size_(data->size_x, data->size_y);

  _collision_checker.setMapSize(resolution_, origin_, map_size_);
  _success_checker.setMapSize(resolution_, origin_, map_size_);

  _collision_checker.setStaticMap(data->static_map.map);
  _collision_checker.setPeds(data->peds_info);

  _success_checker.setClusterMap(data->cluster_map.map);
  _success_checker.setVelMap(data->velocity_map.map_x, data->velocity_map.map_y);
  data_ready = true;
}

void evaluator::robot_pose_callback(const nav_msgs::Odometry &data)
{
  double x_z = data.pose.pose.position.x;
  double y_z = data.pose.pose.position.y;
  double yaw_z = calculate_rpy_from_quat(data.pose.pose.orientation);
  Vector3d robot_pos_(x_z, y_z, yaw_z);
  _collision_checker.setRobotPos(robot_pos_);
  _success_checker.setRobotPos(robot_pos_);
}

void evaluator::timer_callback(const ros::TimerEvent &)
{
  //std::cout <<"hhh " << std::endl;
  int agent_id = 0;
  Vector2d agent_pos;
  if(! data_ready)
    return;
  _collisionLabel = _collision_checker.checkCollision(agent_id, agent_pos);
  _taskCompleteLabel = _success_checker.isSuccess();
  _TASKSTOPLABEL = _collisionLabel || _taskCompleteLabel;
  if(_TASKSTOPLABEL){
    evaluation::evaluation_sending eva_sd_msg;
    eva_sd_msg.collision_label.data = _collisionLabel;
    eva_sd_msg.success_label.data = _taskCompleteLabel;
    eva_sd_msg.pedestrian_id = agent_id;
    eva_sd_msg.pedestrian_pose.x = agent_pos[0];
    eva_sd_msg.pedestrian_pose.y = agent_pos[1];
    _evaluation_pub.publish(eva_sd_msg);

    // do something to stop the simulation
    // tell the system if success or false
    // if success, output to the qt terminal to tell us we succeed
    // if false, tell us that which pedestrian we collide
  }
}

double evaluator::calculate_rpy_from_quat(geometry_msgs::Quaternion q)
{
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  double angle = std::atan2(siny_cosp, cosy_cosp);
  return angle;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluation");
  ROS_INFO("Hi");
  const ros::NodeHandle& nh = ros::NodeHandle("~");
  evaluator eValuator(nh);
  ros::spin();
  return 0;
}
