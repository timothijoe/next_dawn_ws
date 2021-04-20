#include "riskrrt.h"

RRT::RRT(Params params)
{

}

void RRT::init()
{
  //creating a new node at the robot current location, with its current speed
  Node* node;
  node = new Node;
  node->time = ros::Time::now();
  node->pose = robot_pose;
  node->vel = robot_vel;
  node->parent = NULL;
  node->sons.clear();
  node->possible_controls = discretizeVelocities(node);
  node->isOpen = true;
  node->depth = 0;
  node->risk = computeNodeRisk(node);
  node->isFree = (node->risk <= params.threshold);
  //set this new node as the tree root
  root = node;
  //set root as the best node to reach the final goal (this is needed to set the window in order to choose the next random goal)
  best_node = node;
  //the tree root is the first candidate to grow
  candidate_nodes.clear();
  candidate_nodes.push_back(root);


}

vector<Control> RRT::discretizeVelocities(Node *node)
{
    vector<Control> controls;
    Control control;
    double min_linear_vel;
    double max_linear_vel;
    double min_angular_vel;
    double max_angular_vel;
    int i, j;
    double delta_linear, delta_angular;

    //compute maximum and minimum velocities that can be reached from that node given the speed and acceleration limits of the robot
    min_linear_vel = node->vel.linear.x - params.accMax * params.timeStep;
    max_linear_vel = node->vel.linear.x + params.accMax * params.timeStep;
    min_angular_vel = node->vel.angular.z - params.accOmegaMax * params.timeStep;//right
    max_angular_vel = node->vel.angular.z + params.accOmegaMax * params.timeStep;//left

    //make sure that speed limits are not exceeded
    min_linear_vel = max(params.vMin, min_linear_vel);
    max_linear_vel = min(params.vMax, max_linear_vel);
    min_angular_vel = max(-params.omegaMax, min_angular_vel);
    max_angular_vel = min(params.omegaMax, max_angular_vel);

    //create the set of controls
    delta_linear = (max_linear_vel - min_linear_vel) / double(params.nv);
    delta_angular = (max_angular_vel - min_angular_vel) / double(params.nphi);
    for(i=0; i<params.nv; i++){
      control.twist.linear.x = min_linear_vel + i * delta_linear;
      for(j=0; j<params.nphi; j++){
        control.twist.angular.z = min_angular_vel + j * delta_angular;
        control.open = true;
        controls.push_back(control);
      }
    }

    return controls;

}

double RRT::computeNodeRisk(Node *node)
{
    //robot's footprint is assumed to be a rectangle
    custom_pose front_left;
    custom_pose front_right;
    custom_pose rear_left;
    custom_pose rear_right;
      int grid_front_left_x, grid_front_left_y;
      int grid_front_right_x, grid_front_right_y;
      int grid_rear_left_x, grid_rear_left_y;
      int grid_rear_right_x, grid_rear_right_y;
      double l, w;
    double node_theta;
      vector<int> grid_cells;
    int i, j;
    double risk, max_risk;
    int grid_max_x, grid_max_y, grid_min_x, grid_min_y;

    //the wheel axis is assumed to be in the middle
    l = params.robotLength/2.0;
    w = params.robotWidth/2.0;

      //computing the poses of each corner of the robot's footprint
      front_left.x = node->pose.x + (l * cos(node->pose.theta) + w * cos(node->pose.theta + M_PI/2.0));
      front_left.y = node->pose.y + (l * sin(node->pose.theta) + w * sin(node->pose.theta + M_PI/2.0));
      front_right.x = node->pose.x + (l * cos(node->pose.theta) + w * cos(node->pose.theta - M_PI/2.0));
      front_right.y = node->pose.y + (l * sin(node->pose.theta) + w * sin(node->pose.theta - M_PI/2.0));
      rear_left.x = node->pose.x + (l * cos(node->pose.theta + M_PI) + w * cos(node->pose.theta  + M_PI - M_PI/2.0));
      rear_left.y = node->pose.y + (l * sin(node->pose.theta + M_PI) + w * sin(node->pose.theta  + M_PI - M_PI/2.0));
      rear_right.x = node->pose.x + (l * cos(node->pose.theta + M_PI) + w * cos(node->pose.theta + M_PI + M_PI/2.0));
      rear_right.y = node->pose.y + (l * sin(node->pose.theta + M_PI) + w * sin(node->pose.theta + M_PI + M_PI/2.0));

      //the corners poses in grid coordinates
    grid_front_left_x = gridIFromPose(front_left);
    grid_front_left_y = gridJFromPose(front_left);
    grid_front_right_x = gridIFromPose(front_right);
    grid_front_right_y = gridJFromPose(front_right);
    grid_rear_left_x = gridIFromPose(rear_left);
    grid_rear_left_y = gridJFromPose(rear_left);
    grid_rear_right_x = gridIFromPose(rear_right);
    grid_rear_right_y = gridJFromPose(rear_right);

      //testing: simple bounding box TODO: exact footprint
      grid_max_x = max(grid_front_left_x, max(grid_front_right_x, max(grid_rear_left_x, grid_rear_right_x)));
      grid_max_y = max(grid_front_left_y, max(grid_front_right_y, max(grid_rear_left_y, grid_rear_right_y)));
      grid_min_x = min(grid_front_left_x, min(grid_front_right_x, min(grid_rear_left_x, grid_rear_right_x)));
      grid_min_y = min(grid_front_left_y, min(grid_front_right_y, min(grid_rear_left_y, grid_rear_right_y)));

    //creating a list of all the grid cells within the robot's footprint
      for(i=grid_min_x ; i<=grid_max_x ; i++){
          for(j=grid_min_y ; j<=grid_max_y ; j++){
              if (i >=0 && i < (int)og_array.array[0].info.width && j >=0 && j< (int)og_array.array[0].info.height){
                  grid_cells.push_back(gridIndexFromCoord(i,j));
              }
          }
      }

    //going through all the cells in robot footprint and getting the maximum risk
    max_risk = 0.0;
    for(i=0; i<grid_cells.size(); i++){
      risk = og_array.array[node->depth].data[grid_cells[i]];
      max_risk = max(risk, max_risk);
    }

    //risk propagation from a node to his sons if their risk is lower
    if(node->parent != NULL){
      max_risk = max(max_risk, node->parent->risk);
    }

      return max_risk;


}

void RRT::initcontrollerFeedbackSub()
{
    controllerFeedbackSubscriber = nodeHandle.subscribe("/controller_feedback", 1, &RRT::controllerFeedbackCallback, this);
}

void RRT::initOgArraySub()
{
    ogArraySubscriber = nodeHandle.subscribe("/ogarray", 1, &RRT::ogArrayCallback, this);
}

void RRT::initOdomSub()
{
    odomSubscriber = nodeHandle.subscribe("/odom", 1, &RRT::odomCallback, this);
}

void RRT::robot_pose_callback(const nav_msgs::Odometry &data)
{
    double x_z = data.pose.pose.position.x;
    double y_z = data.pose.pose.position.y;
    double yaw_z = tf::getYaw(data.pose.pose.orientation);
    //double yaw_z = calculate_rpy_from_quat(data.pose.pose.orientation);
    robot_pose.x = x_z;
    robot_pose.y = y_z;
    robot_pose.theta = yaw_z;
}

void RRT::initPoseSub()
{
    //poseSubscriber = nodeHandle.subscribe("/amcl_pose", 1, &RRT::poseCallback, this);
    _robot_pose_sub = nodeHandle.subscribe("/pedsim_simulator/robot_position", 1, &RRT::robot_pose_callback, this);
}

void RRT::initGoalSub()
{
    goalSubscriber = nodeHandle.subscribe("/goal", 1, &RRT::goalCallback, this);
}
