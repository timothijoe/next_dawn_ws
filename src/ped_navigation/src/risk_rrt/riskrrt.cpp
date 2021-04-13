#include "risk_rrt/riskrrt.h"

RiskRRT::RiskRRT()
{

}

void RiskRRT::init()
{
  Node* node;
  node = new Node;
  node->time = ros::Time::now();
  node->pose = robot_pose;
  node->vel = robot_vel;
  node->parent = NULL;
  node->sons.clear();
  node->possible_controls = discretizeVelocities(node);
  node->isOpen = True;
  node->depth = 0;
  node->risk = computeNodeRisk(node);
  node->isFree = (node->risk <= params.threshold);
  root = node;
  best_node = node;
  candidate_nodes.clear();
  candidate_nodes.push_back(root);

}

vector<Control> RiskRRT::discretizeVelocities(Node *node)
{
  vector<Control> controls;
  Control control;
  double min_linear_vel;
  double max_linear_vel;
  double min_angular_vel;
  double max_angular_vel;
  int i, j;
  double delta_linear, delta_angular;

  min_linear_vel = node->vel.linear.x - params.accMax * params.timeStep;
  max_linear_vel = node->vel.linear.x + params.accMax * params.timeStep;
  min_angular_vel = node->vel.angular.z - params.accOmegaMax * params.timeStep;
  max_angular_vel = node->vel.angular.z - params.accOmegaMax * params.timeStep;

  min_linear_vel = max(params.vMin, min_linear_vel);
  max_linear_vel = min(params.vMax, max_linear_vel);
  min_angular_vel = max(-params.omegaMax, min_angular_vel);
  max_angular_vel = min(params.omegaMax, max_angular_vel);

  // create the set of controls
  delta_linear = (max_linear_vel - min_linear_vel) / double(params.nv);
  delta_angular = (max_angular_vel - min_angular_vel) / double(params.nphi);
  for(int i = 0; i < params.nv; i++){
    control.twist.linear.x = min_linear_vel + i * delta_linear;
    for(j = 0; j < params.nphi; j++){
      control.twist.angular.z = min_angular_vel + j * delta_angular;
      control.open = true;
      controls.push_back(control);
    }
  }
  return controls;

}

double RiskRRT::computeNodeRisk(Node *node)
{
  custom_pose front_left;
  custom_pose front_right;
  custom_pose rear_left;
  custom_pose rear_right;
  int grid_front_left_x, grid_front_left_y;
  int grid_front_right_x, grid_front_right_y;
  int grid_rear_left_x, grid_rear_left_y;
  int grid_rear_right_x, grid_rear_right_y;
  double l,w;
  double node_theta;
  vector<int> grid_cells;
  int i, j;
  double risk, max_risk;
  int grid_max_x, grid_max_y, grid_min_x, grid_min_y;

  l = params.robotLength/ 2.0;
  w = params.robotWidth / 2.0;

  front_left.x = node->pose.x + (l * cos(node->pose.theta) + w * cos(node->pose.theta + M_PI / 2.0));
  front_left.y = node->pose.y + (l * sin(node->pose.theta) + w * sin(node->pose.theta + M_PI / 2.0));
  front_right.x = node->pose.x + (l * cos(node->pose.theta) + w * cos(node->pose.theta - M_PI / 2.0));
  front_right.y = node->pose.y + (l * sin(node->pose.theta) + w * sin(node->pose.theta - M_PI / 2.0));
  rear_left.x = node->pose.x + (l * cos(node->pose.theta + M_PI) + w * cos(node->pose.theta + M_PI - M_PI / 2.0));
  rear_left.y = node->pose.y + (l * sin(node->pose.theta + M_PI) + w * sin(node->pose.theta + M_PI - M_PI / 2.0));
  rear_right.x = node->pose.x + (l * cos(node->pose.theta + M_PI) + w * cos(node->pose.theta + M_PI + M_PI / 2.0));
  rear_right.y = node->pose.y + (l * sin(node->pose.theta + M_PI) + w * sin(node->pose.theta + M_PI + M_PI / 2.0));

  grid_front_left_x =











}
