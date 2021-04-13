#ifndef COHERENT_FILTER_H
#define COHERENT_FILTER_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>
#include <map_generator/ped_xvi.h>
#include <map>
using namespace std;
using namespace Eigen;

class coherentFilter
{
public:
  coherentFilter();
  void reset_cur_peds(vector<single_ped_xvi> z_cur_peds_xvi);
  void get_neighbor();
  void pair_to_cluster();
  void reidentify();
  map<int, int> get_cluster(vector<single_ped_xvi> z_cur_peds_xvi);
  int zt =3;

  vector<single_ped_xvi> cur_peds_xvi;

  map<int,single_ped_xvi> _ego_dict;
  map<int,vector<int>> _neighbor_dict;

  map<int, vector<int>> _cluster_inverse_index;
  map<int, int> _cluster_index;
  map<int, int> re_cluster_index;
};

#endif // COHERENT_FILTER_H
