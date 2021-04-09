#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>
#include <riskrrt/OccupancyGridArray.h>
#include <riskrrt/PoseTwistStamped.h>
#include <riskrrt/Trajectory.h>
#include <iostream>
#include <vector>

#include <tf/tf.h>
#include <algorithm>
#include <visualization_msgs/MarkerArray.h>
#include <pedsim_msgs/AgentStates.h>

using namespace std;

struct custom_pose{//ros pose msgs use quaternion and the function provided by the tf library to get yaw from quaternion is terrible, hence this.
  double x;//in meters
  double y;//in meters
  double theta;//in radians (-PI, PI)
};

struct pedestrian_pose{//ros pose msgs use quaternion and the function provided by the tf library to get yaw from quaternion is terrible, hence this.
  double px;
  double py;
  double vx;
  double vy;
};

nav_msgs::OccupancyGrid grid;
//nav_msgs::OccupancyGrid modified_grid;
riskrrt::OccupancyGridArray og_array;
int nbMap;
double timeStep;
//custom_pose temp_pose, ped_pose;

int gridIFromPose(custom_pose pose){
  return (int)round((pose.x - grid.info.origin.position.x) / grid.info.resolution);
}

int gridJFromPose(custom_pose pose){
  return (int)round((pose.y - grid.info.origin.position.y) / grid.info.resolution);
}

int gridIndexFromCoord(int i, int j){
  return i + grid.info.width * j;
}

int gridIFromIndex(int index){
  return  index % grid.info.width;
}

int gridJFromIndex(int index){
  return floor(index / grid.info.width);
}

custom_pose poseFromGridCoord(int i, int j){
  custom_pose pose;
  pose.x = grid.info.resolution * i + grid.info.origin.position.x;
  pose.y = grid.info.resolution * j + grid.info.origin.position.y;
  return pose;
}

int gridIndexFromPose(custom_pose pose){
  int index, i, j;
  i = gridIFromPose(pose);
  j = gridJFromPose(pose);
  index = gridIndexFromCoord(i, j);
  return index;
}

void ogmap_clear(){
  og_array.array.clear();
  //nav_msgs::OccupancyGrid grid;
  grid.info.resolution = 0.3; // 0.054
  grid.info.width = int (40 / grid.info.resolution);
  grid.info.height = int (18 / grid.info.resolution);
  for(int i=0; i < nbMap; i++){
    og_array.array.push_back(grid);
  }
}

void pedestrian_callback(const pedsim_msgs::AgentStatesPtr data)
{
   ogmap_clear();
   for(auto it=data->agent_states.begin();it!=data->agent_states.end(); ++it){
     pedestrian_pose ped_pose;
     int i, j, k;
     int ped_grid_index;
     int ped_grid_i;
     int ped_grid_j;

     int max_i, min_i;
     int max_j, min_j;

     ped_pose.px = it->pose.position.x;
     ped_pose.py = it->pose.position.y;
     ped_pose.vx = it->twist.linear.x;
     ped_pose.vy = it->twist.linear.y;

     for(int k = 0; k < nbMap; k++){
       double temp_pose_x = ped_pose.px + k * timeStep * ped_pose.vx;
       double temp_pose_y = ped_pose.px + k * timeStep * ped_pose.vy;
       custom_pose temp_pose;
       temp_pose.x = temp_pose_x;
       temp_pose.y = temp_pose_y;
       ped_grid_i = gridIFromPose(temp_pose);
       ped_grid_j = gridJFromPose(temp_pose);

       min_i = max(ped_grid_i - 5, 0);
       max_i = min(ped_grid_i + 5, (int)grid.info.width);
       min_j = max(ped_grid_j - 5, 0);
       max_j = min(ped_grid_j + 5, (int)grid.info.height);

       for(i=min_i ; i<=max_i ; i++){for(j=min_j ; j<=max_j ; j++){og_array.array[k].data[gridIndexFromCoord(i,j)] = 100;}}
     }
   }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "og_builder_stage");
  ros::NodeHandle n;

  n.param("maxDepth", nbMap, 10);
  n.param("timeStep", timeStep, 0.5);

  ros::Subscriber _ped_subscriber = n.subscribe("/pedsim_simulator/simulated_agents", 1,pedestrian_callback);
  //ros::Publisher test_pub = n.advertise<nav_msgs::OccupancyGrid>("test_map", 1);
  ros::Publisher og_pub = n.advertise<riskrrt::OccupancyGridArray>("ogarray", 1);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    og_pub.publish(og_array);
    loop_rate.sleep();
  }
  return 0;
}
