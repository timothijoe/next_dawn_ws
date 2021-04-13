#ifndef COMINTERFACE_H
#define COMINTERFACE_H

#include <ros/ros.h>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <pedsim_srvs/ResetScene.h>
#include <pedsim_srvs/ResetRobotPos.h>
#include "evaluation/evaluation_sending.h"

//class ControlPannel;
class ComInterface : public QThread{
  Q_OBJECT
public:
  // Function
  ComInterface();
  ComInterface(int argc, char** argv);
  virtual ~ComInterface();
  bool init();
  void run();
  void subCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void evaSubCallback(const evaluation::evaluation_sending::ConstPtr& msg);
  bool callService();
  bool callRestartService(bool default_pos_label, int random_seed_id, std::string scene_name, std::vector<double> robot_pos);
  bool callStopService();
  bool callContinueService();
  bool setRobotStop();
  bool setRobotContinue();
  bool setRobotPos(bool label, double r_x, double r_y, double r_theta);
  bool setTargetPos(double r_x, double r_y, double r_theta);
  //bool exe_service_linzong();
  //ros::ServiceClient client;

Q_SIGNALS:
  void figureUpdated(double data1, double data2);
  void pedCollision(int fail_times_);
  void taskCompleted(int success_times_);
  void rosShutdown();

private:
  int init_argc;
  char** init_argv;
  int total_experiments_needed = 30;
  int success_times = 0;
  int failed_times = 0;
  int total_times = 0;
  //ControlPannel *control_pannel;
  ros::Publisher chatter_publisher;
  ros::Subscriber data_sub;
  ros::Subscriber eva_sub;
  ros::ServiceClient client;
  ros::ServiceClient restart_client;
  ros::ServiceClient stop_client;
  ros::ServiceClient continue_client;
  ros::ServiceClient robot_stop_continue_client;
  ros::ServiceClient reset_robot_client_;
  ros::ServiceClient reset_target_client_;


};

#endif // COMINTERFACE_H
