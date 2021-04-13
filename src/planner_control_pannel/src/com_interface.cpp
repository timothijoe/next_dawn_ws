#include "include/com_interface.h"
#include <ros/ros.h>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <unistd.h>
//#include "control_pannel.h"
ComInterface::ComInterface()
{}


ComInterface::ComInterface(int argc, char **argv):
    init_argc(argc),
    init_argv(argv)
 {}


ComInterface::~ComInterface()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}


bool ComInterface::init()
{
  ros::init(init_argc,init_argv,"showcurve");
    if ( ! ros::master::check() ) {
      return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    data_sub = n.subscribe("/Frame_loss", 1000, &ComInterface::subCallback, this);
    eva_sub = n.subscribe("/evaluation_information", 1, &ComInterface::evaSubCallback, this);
    client = n.serviceClient<std_srvs::SetBool>("/learn_srvs");
    robot_stop_continue_client = n.serviceClient<std_srvs::SetBool>("/continue_stop_robot_service");
    restart_client = n.serviceClient<pedsim_srvs::ResetScene>("/pedsim_simulator/restart_scenario");
    stop_client = n.serviceClient<std_srvs::Empty>("/pedsim_simulator/pause_simulation");
    continue_client = n.serviceClient<std_srvs::Empty>("/pedsim_simulator/unpause_simulation");
    reset_robot_client_ = n.serviceClient<pedsim_srvs::ResetRobotPos>("/pedsim_simulator/reset_robot_pos");
    reset_target_client_ = n.serviceClient<pedsim_srvs::ResetRobotPos>("/ped_navigation/reset_target_pos");
    start();
    return true;
}


void ComInterface::run()
{
  ros::Rate loop_rate(1);
  while ( ros::ok() ) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();
}


void ComInterface::subCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  double data1 = msg->data[0];
  double data2 = msg->data[1];
  Q_EMIT figureUpdated(data1,data2);
}

void ComInterface::evaSubCallback(const evaluation::evaluation_sending::ConstPtr &msg)
{
  bool collision_label = msg->collision_label.data;
  bool success_label = msg->success_label.data;
  int pedestrian_id = msg->pedestrian_id;
  double p_x = msg->pedestrian_pose.x;
  double p_y = msg->pedestrian_pose.y;
  if(collision_label == true){
//    failed_times += 1;
    Q_EMIT pedCollision(failed_times);
  }
  if(success_label == true){
//    success_times += 1;
     Q_EMIT taskCompleted(success_times);
  }
  return;


}

bool ComInterface::callService()
{
  //std::cout<< " ge ming bi sheng "<<std::endl;
  ros::service::waitForService("/learn_srvs",-1);
  std_srvs::SetBool srv;
  if(client.call(srv))
  {
    ROS_INFO("Success");
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
  }
}

bool ComInterface::callRestartService(bool default_pos_label, int random_seed_id, std::string scene_name, std::vector<double> robot_pos)
{
  pedsim_srvs::ResetScene srv;
  srv.request.random_seed_id = random_seed_id;
  srv.request.scene_name = scene_name;
  srv.request.use_default_robot_position = default_pos_label;
  srv.request.robot_pos = robot_pos;
  if(restart_client.call(srv))
  {
    ROS_INFO("Success");
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
  }
}

bool ComInterface::callStopService()
{
  // Second step, stop the robot
  ros::service::waitForService("/continue_stop_robot_service",-1);
  std_srvs::SetBool srvr;
  srvr.request.data = true;
  if(robot_stop_continue_client.call(srvr))
  {
    ROS_INFO("Success to stop the robot moving ~");
  }
  else
  {
    ROS_ERROR("Failed to stop the robot moving!");
  }
  sleep(0.2);
  // First step, stop the pedestrians
  ros::service::waitForService("/pedsim_simulator/pause_simulation", -1);
  std_srvs::Empty srv;
  if(stop_client.call(srv))
  {
    ROS_INFO("Success to stop the pedestrians moving ~");
  }
  else
  {
    ROS_ERROR("Failed to stop the pedestrians moving!");
  }
}

bool ComInterface::callContinueService()
{
  ros::service::waitForService("/pedsim_simulator/unpause_simulation", -1);

  std_srvs::Empty srv;
  if(continue_client.call(srv))
  {
    ROS_INFO("Success to continue the pedestrians moving ~");
  }
  else
  {
    ROS_ERROR("Failed to continue the pedestrians moving!");
  }
  sleep(0.4);
  ros::service::waitForService("/continue_stop_robot_service",-1);
  std_srvs::SetBool srvr;
  srvr.request.data = false;
  if(robot_stop_continue_client.call(srvr))
  {
    ROS_INFO("Success to continue the robot moving ~");
  }
  else
  {
    ROS_ERROR("Failed to continue the robot moving!");
  }


}

bool ComInterface::setRobotStop()
{
  ros::service::waitForService("/continue_stop_robot_service",-1);
  std_srvs::SetBool srv;
  srv.request.data = true;
  if(robot_stop_continue_client.call(srv))
  {
    ROS_INFO("Success");
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
  }
}

bool ComInterface::setRobotContinue()
{
  ros::service::waitForService("/continue_stop_robot_service",-1);
  std_srvs::SetBool srv;
  srv.request.data = false;
  if(robot_stop_continue_client.call(srv))
  {
    ROS_INFO("Success");
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
  }
}

bool ComInterface::setRobotPos(bool label, double r_x, double r_y, double r_theta)
{
  //reset_robot_client_ = nh_.serviceClient<pedsim_srvs::ResetRobotPos>("/pedsim_simulator/reset_robot_pos");
  ros::service::waitForService("/pedsim_simulator/reset_robot_pos");
  sleep(0.2);
  pedsim_srvs::ResetRobotPos srv;
  srv.request.use_default_robot_position = label;
  srv.request.robot_pos.push_back(r_x);
  srv.request.robot_pos.push_back(r_y);
  srv.request.robot_pos.push_back(r_theta);
  if(reset_robot_client_.call(srv))
  {
    ROS_INFO("Success");
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
  }

}

bool ComInterface::setTargetPos(double r_x, double r_y, double r_theta)
{
  ros::service::waitForService("/ped_navigation/reset_target_pos");
  sleep(0.2);
  pedsim_srvs::ResetRobotPos srv;
  srv.request.robot_pos.push_back(r_x);
  srv.request.robot_pos.push_back(r_y);
  srv.request.robot_pos.push_back(r_theta);
  if(reset_target_client_.call(srv))
  {
    ROS_INFO("Success");
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
  }
}










