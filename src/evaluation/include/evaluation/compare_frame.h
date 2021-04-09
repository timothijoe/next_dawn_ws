#ifndef COMPARE_FRAME_H
#define COMPARE_FRAME_H
#include <ros/console.h>
#include <ros/ros.h>
#include <functional>
#include <iostream>
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/Eva_Info.h>
#include "evaluation/computation_geometry_utils.h"
#include <std_srvs/Empty.h>
#include <math.h>
#include <bits/stdc++.h>
#include "xml_utils/tinyxml.h"

#include "map_generator/ped_xvi.h"
#include "map_generator/quad_tree.h"

using namespace std;
using namespace Eigen;


class compareFrame
{
public:
  enum CompareExe{
    LOAD,
    WRITE
  };
  class singleFrame{
  public:
    int frame_id;
    vector<single_ped_xvi> agents_list;
    singleFrame(int id=0):frame_id{id} {}
  };

  class singleEntropy{
  public:
    int frame_id;
    double difference;
  };

  class totalEntropy{
  public:
    int last_frame_id;
    double t_difference = 0;
    vector<singleEntropy> entropy_list;
  };

  compareFrame(ros::NodeHandle nh);
  void data_process(const pedsim_msgs::Eva_InfoPtr& data);
  void timer_callback(const ros::TimerEvent&);
  double calculate_rpy_from_quat(geometry_msgs::Quaternion q);
  void write_agent_traj_to_xml();
  void load_agent_traj_from_xml();
  void calc_cur_entropy(singleFrame current_frame_);
  bool getAgent_fromFrame_by_id(uint64_t id_, single_ped_xvi agent_, singleEntropy frame_);
private:
  CompareExe _exetype = LOAD;
  vector<singleFrame> _std_frame_list;
  vector<singleFrame> _cur_frame_list;
  int std_frame_capacity;
  int cur_frame_capacity;

  totalEntropy _overall_entropy;
  // current robot and pedestrian information
  Vector3d _robot_pos;
  singleFrame current_frame;

  // subscriber to check
  ros::Subscriber env_eval_info_sub;  // pedestrian subscriber
  ros::Timer timer;




};

#endif // COMPARE_FRAME_H
