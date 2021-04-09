#include "evaluation/collision_checker.h"

CollisionChecker::CollisionChecker(ros::NodeHandle nh)
{
  _ped_subscriber = nh.subscribe("/pedsim_simulator/simulated_agents", 1, &CollisionChecker::pedestrian_callback, this);
  robot_pose_sub = nh.subscribe("/pedsim_simulator/robot_position", 1, &CollisionChecker::robot_pose_callback, this);
  _timer = nh.createTimer(ros::Duration(0.1), &CollisionChecker::timer_callback, this);
  _update_shutdown_timer_ = nh.createTimer(ros::Duration(3), &CollisionChecker::update_shutdown_parameter, this);
  stop_client = nh.serviceClient<std_srvs::Empty>("/pedsim_simulator/pause_simulation");
  robot_stop_continue_client = nh.serviceClient<std_srvs::SetBool>("/continue_stop_robot_service");
  quad_tree.setDim(0, 0, 0, 35, 18);
}

void CollisionChecker::pedestrian_callback(const pedsim_msgs::AgentStatesPtr data)
{
  if(!exec_ready_label)
    return;
  data_ready_label = false;
  cur_peds_xvi.clear();
  for (auto it = data->agent_states.begin(); it != data->agent_states.end(); ++it){
    single_ped_xvi cur_ped_xvi = single_ped_xvi(*it);
    cur_peds_xvi.push_back(cur_ped_xvi);
  }
  data_ready_label = true;
}


// collision checking part, we first push all pedestrians into the quadtree and then check the distance with the robot.
void CollisionChecker::timer_callback(const ros::TimerEvent &)
{
  if(!data_ready_label)
    return;
  if(cur_peds_xvi.empty())
    return;
  //std::cout << "hh1" <<std::endl;
  exec_ready_label = false;

  quad_tree.get_tree_from_list(cur_peds_xvi);
  //vector<single_ped_xvi> near_ped_list = cur_peds_xvi;
  vector<single_ped_xvi> near_ped_list;
  quad_tree.getNeighbors(near_ped_list, robot_position[0], robot_position[1], dist_safe);
 // std::cout << "hhh2       "<<std::endl;
  if(!near_ped_list.empty()){
  for(auto ped : near_ped_list){
    Vector2d rp(robot_position[0],robot_position[1]);
    Vector2d diff = ped.agent_pos - rp;
    if(diff.norm() < dist_safe){
      collision_mem_id = ped.agent_id;
      std::cout << "collision happens" << std::endl;

      post_collision_function();
      break;
    }

  }
}

  exec_ready_label = true;

}

void CollisionChecker::update_shutdown_parameter(const ros::TimerEvent &)
{
  system_shutdown_label = false;

}

void CollisionChecker::robot_pose_callback(const nav_msgs::Odometry &data)
{
  if(!exec_ready_label)
    return;
  double x_z = data.pose.pose.position.x;
  double y_z = data.pose.pose.position.y;
  double yaw_z = calculate_rpy_from_quat(data.pose.pose.orientation);
  robot_position << x_z, y_z, yaw_z;
}

double CollisionChecker::calculate_rpy_from_quat(geometry_msgs::Quaternion q)
{
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  double angle = std::atan2(siny_cosp, cosy_cosp);
  return angle;
}

void CollisionChecker::post_collision_function()
{
  // if non ignore, we will stop the simulation to see what happens
  if(!ignore_collision_label){
    if(system_shutdown_label){
      return;
    }
    //std::cout << " ge ming bi sheng " << std::endl;
    // call the function to stop the simulation;
    ros::service::waitForService("/pedsim_simulator/pause_simulation",-1);
    std_srvs::Empty srv;
    if(stop_client.call(srv))
    {
      ROS_INFO("Success");
    }
    else
    {
      ROS_ERROR("Failed to call service add_two_ints");
    }

    ros::service::waitForService("/continue_stop_robot_service",-1);
    std_srvs::SetBool srv1;
    srv1.request.data = true;
    if(robot_stop_continue_client.call(srv1))
    {
      ROS_INFO("Success");
    }
    else
    {
      ROS_ERROR("Failed to call service add_two_ints");
    }
    system_shutdown_label = true;

  }
  // if we not stop the post collision function.
  std::cout<< "collision happens, the collision id is : " << collision_mem_id<< "hhh" << std::endl;
}

