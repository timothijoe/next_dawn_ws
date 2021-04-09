#ifndef PED_XVI_H
#define PED_XVI_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>
#include <map>
using namespace std;
using namespace Eigen;

// The first class, containing the id, pos, velocity and cluster id.
class single_ped_xvi{
public:
  single_ped_xvi(pedsim_msgs::AgentState agent_state){
    agent_id = (int)agent_state.id;
    agent_pos << agent_state.pose.position.x, agent_state.pose.position.y;
    agent_vel << agent_state.twist.linear.x,agent_state.twist.linear.y;
    cluster_id = (int)agent_state.cluster_id;
    //std::cout << " cluster id is : " << std::endl;
  }
  single_ped_xvi(int agent_id_, double x_, double y_):agent_id(agent_id_), agent_pos(Vector2d(x_, y_)){}
  int agent_id;
  int cluster_id;
  Vector2d agent_pos;
  Vector2d agent_vel;
  double agent_dist = 0;
};


// The second class, used to calculate neighbor for each pedestrian
class certain_ped_neighbor{
public:
  certain_ped_neighbor(single_ped_xvi cur_ped_xvi, int KNN_ = 10){
    ego_id = cur_ped_xvi.agent_id;
    ego_position << cur_ped_xvi.agent_pos[0], cur_ped_xvi.agent_pos[1];
    ego_velocity << cur_ped_xvi.agent_vel[0], cur_ped_xvi.agent_vel[1];
    KNN = KNN_;
  }
  double calc_dist(single_ped_xvi other_ped_xvi){
    Vector2d pos_err = ego_position - other_ped_xvi.agent_pos;
    return pos_err.norm();
  }
  double calc_correlation(single_ped_xvi other_ped_xvi){
    Vector2d other_velocity(other_ped_xvi.agent_vel[0],other_ped_xvi.agent_vel[1]);
    double ego_speed = ego_velocity.norm();
    double other_speed = other_velocity.norm();
    double correlation = 0;
    if((ego_speed > 0) && (other_speed > 0)){
      double vel_err = ego_velocity[0]*other_velocity[0] + ego_velocity[1]*other_velocity[1];
      correlation = vel_err/(ego_speed * other_speed);
    }
    return correlation;
  }
  vector<int> sort(vector<single_ped_xvi> cur_peds_xvi){
    vector<single_ped_xvi> s;
    for(auto& cur_neighbor_mem :cur_peds_xvi ){
      double dist = this->calc_dist(cur_neighbor_mem);
      if( (cur_neighbor_mem.agent_id == this->ego_id) || (dist>2.5) ){
        continue;}
      else{
        cur_neighbor_mem.agent_dist = dist;
        s.push_back(cur_neighbor_mem);}}
        std::sort(s.begin(), s.end(), [](single_ped_xvi i,single_ped_xvi j){
          return i.agent_dist<j.agent_dist;
        });
        //double max_neighbor_num = min(int(s.size()) , int(s.size()));
        double max_neighbor_num = min(this->KNN , int(s.size()));
        std::vector<single_ped_xvi>::const_iterator first1 = s.begin();
        std::vector<single_ped_xvi>::const_iterator last1 = s.begin() + max_neighbor_num;
        std::vector<single_ped_xvi> cur_peds_sorted(first1, last1);
        //vector<single_ped_xvi> cur_neighbor_with_cor;
        vector<int> cur_neighbor_with_cor;
        for(auto& neighbor: cur_peds_sorted){
          double corre = this->calc_correlation(neighbor);
          if(corre >= 0.6){
            //neighbor.agent_corre = corre;
            cur_neighbor_with_cor.push_back(neighbor.agent_id);
          }
        }
     return cur_neighbor_with_cor;
  }
  // class member
  int ego_id;
  Vector2d ego_position;
  Vector2d ego_velocity;
  int KNN;
};


#endif // PED_XVI_H
