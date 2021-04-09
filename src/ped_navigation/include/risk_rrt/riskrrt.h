#ifndef RISKRRT_H
#define RISKRRT_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <tf/tf.h>


#include "math.h"
#include <Eigen/Eigen>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ctime>
#include <chrono>
#include "flowsegments.h"

// ROS location
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int16.h>

#include "lattice/map_receiver.h"

#include "lattice/diff_drive_generator/motion_model_diff_drive.h"
#include "lattice/diff_drive_generator/trajectory_generator_diff_drive.h"
#include "lattice/lattice_lookup_table/lookup_table_utils.h"

//#include "graph/accessor.hpp"
#include "graph/delaunator.hpp"

#include "ped_navigation/m_density_map.h"
#include "ped_navigation/m_static_map.h"
#include "ped_navigation/m_velocity_map.h"
#include "ped_navigation/map_utils.h"
#include "ped_navigation/p_opt_trajectory.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <math.h>

using namespace std;

struct Params{
  double timeStep;  // time step betwwen each node
  int maxDepth; // maximum depth a node can have
  int nv;
  int nphi;
  double threshold; // maximum risk value a node can have to be considered free
  double socialWeight; // weight of risk component in the computetaion of the node
  double rotationWeight; // weight of the node's orientation toward the goal
  double growTime; // time allocated for the tree growth between the map updates
  double bias; // percentage of the time the final goal is chosen as random goal
  double goalTh; // maximum distance the robot can be from the final goal to consider the goal reach
  double windowSize; // sieze of the window from which to pick a random goal
  double robotLength; // robot length, the wheel axis is assumed to be in the middle
  double robotWidth; // robot width;
  double vMin; // minimum robot linear speed (m/s), set it to 0 or a negative value to allow reverse
  double vMax; // maximum robot linear speed (m/s)
  double accMax; // maximum linear acceleration
  double omegaMax; // maximum angular speed (rad/s)
  double accOmegaMax; // maimum angular acceleration
};

struct custom_pose{
  double x;
  double y;
  double theta;
};

struct Control{
  geometry_msgs::Twist twist;
  bool open; // whether the control has already been tested or not
  // (does not mmater if the resulting node was in collision or was effectively added to the tree)
};

struct Node{
  ros::Time time; // time at which node is valid
  custom_pose pose; // pose of the node
  geometry_msgs::Twist vel;
  Node* parent; // parent node
  vector<Node*> sons; // list of sons
  vector<Control> possible_controls; // list of all possible controls from that node considering nv, nphi, velocities and acceleration limits
  double risk; // node risk (not equal to node weight, we don't consider distance to goal)
  int depth; // depth of node in the tree
  bool isFree; // false if the node is in collision(the risk is higher than the threshold set by user)
  bool isOpen; // true if at least one control among all possible controls remain open
};


class RiskRRT
{
public:
  RiskRRT();
  ros::NodeHandle nodeHandle;
  custom_pose robot_pose; // pose of the robot
  geometry_msgs::Twist robot_vel;
  Node* root; // tree root
  Node* best_node; // best node to reach the final goal (=/= best node to random goal)
  vector<Node*> candidate_nodes; // all the nodes that are candidates to grow, list is created and updated during growing phase, then deleted
  custom_pose final_goal;
  bool goal_received;
  bool robot_on_traj;
  // riskrrt::OccupancyGridArray og_array;
  // riskrrt::Trajectory traj_msg;
  vector<Node*> traj;
  Params params;
  mapReceiver _map_receiver;



  visualization_msgs::MarkerArray node_markers;
  visualization_msgs::MarkerArray path_markers;
  ros::Subscriber controllerFeedbackSubscriber;
  ros::Subscriber ogArraySubscriber;
  ros::Subscriber poseSubscriber;
  ros::Subscriber goalSubscriber;
  ros::Subscriber odomSubscriber;

  void init();
  void grow();
  void grow_to_goal(Node* best_node, custom_pose goal);
  void update();
  void updateNodes();
  void deleteUnreachableNodes(Node* new_root);
  Node* chooseBestNode(custom_pose goal);
  void findPath();
  bool isGoalReached();
  custom_pose chooseRandomGoal();
  double computeNodeWeight(Node* node, custom_pose goal);
  void extend(Node* node, custom_pose random_goal);
  vector<Control> discretizeVelocities(Node* node);
  double trajLength(custom_pose pose, custom_pose goal);
  custom_pose robotKinematic(custom_pose pose, Control control);
  double computeControlscore(custom_pose expected_pose, custom_pose random_goal);
  double computeNodeRisk(Node* node);




};

#endif // RISKRRT_H
