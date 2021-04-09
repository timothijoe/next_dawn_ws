/**
* Copyright 2014-2016 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
* \author Sven Wehner <mail@svenwehner.de>
*/

#include <QApplication>
#include <algorithm>

#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/simulator.h>

#include <pedsim_utils/geometry.h>
#include <ros/package.h>
using namespace pedsim;

Simulator::Simulator(const ros::NodeHandle& node) : nh_(node) {
  dynamic_reconfigure::Server<SimConfig>::CallbackType f;
  f = boost::bind(&Simulator::reconfigureCB, this, _1, _2);
  server_.setCallback(f);
}

Simulator::~Simulator() {
  // shutdown service servers and publishers
  pub_obstacles_.shutdown();
  pub_agent_states_.shutdown();
  pub_agent_groups_.shutdown();
  pub_robot_position_.shutdown();

  srv_pause_simulation_.shutdown();
  srv_unpause_simulation_.shutdown();

  delete robot_;
  QCoreApplication::exit(0);
}

bool Simulator::initializeSimulation() {
  int queue_size = 0;
  nh_.param<int>("default_queue_size", queue_size, 1);
  ROS_INFO_STREAM("Using default queue size of "
                  << queue_size << " for publisher queues... "
                  << (queue_size == 0
                          ? "NOTE: This means the queues are of infinite size!"
                          : ""));

  // setup ros publishers
  pub_obstacles_ =
      nh_.advertise<pedsim_msgs::LineObstacles>("simulated_walls", queue_size);
  pub_agent_states_ =
      nh_.advertise<pedsim_msgs::AgentStates>("simulated_agents", queue_size);
  pub_agent_groups_ =
      nh_.advertise<pedsim_msgs::AgentGroups>("simulated_groups", queue_size);
  pub_robot_position_ =
      nh_.advertise<nav_msgs::Odometry>("robot_position", queue_size);
  pub_eval_info_ =
      nh_.advertise<pedsim_msgs::Eva_Info>("evaluation_info", queue_size);

  // services
  srv_pause_simulation_ = nh_.advertiseService(
      "pause_simulation", &Simulator::onPauseSimulation, this);
  srv_unpause_simulation_ = nh_.advertiseService(
      "unpause_simulation", &Simulator::onUnpauseSimulation, this);
  srv_remove_all_elements_ = nh_.advertiseService(
      "restart_scenario", &Simulator::restartScenario, this);
  reset_robot_client_ = nh_.serviceClient<pedsim_srvs::ResetRobotPos>("/pedsim_simulator/reset_robot_pos");

  // setup TF listener and other pointers
  transform_listener_.reset(new tf::TransformListener());
  robot_ = nullptr;

  // load additional parameters
  std::string scene_file_param;
  nh_.param<std::string>("scene_file", scene_file_param, "");
  if (scene_file_param == "") {
    ROS_ERROR_STREAM("Invalid scene file: " << scene_file_param);
    return false;
  }

  ROS_INFO_STREAM("Loading scene [" << scene_file_param << "] for simulation");

  const QString scenefile = QString::fromStdString(scene_file_param);
  //ScenarioReader scenario_reader;
  if (scenario_reader.readFromFile(scenefile) == false) {
    ROS_ERROR_STREAM(
        "Could not load the scene file, please check the paths and param "
        "names : "
        << scene_file_param);
    return false;
  }

  nh_.param<bool>("enable_groups", CONFIG.groups_enabled, true);
  nh_.param<double>("max_robot_speed", CONFIG.max_robot_speed, 1.5);
  nh_.param<double>("update_rate", CONFIG.updateRate, 25.0);
  nh_.param<double>("simulation_factor", CONFIG.simulationFactor, 1.0);

  int op_mode = 1;
  nh_.param<int>("robot_mode", op_mode, 1);
  CONFIG.robot_mode = static_cast<RobotMode>(op_mode);

  paused_ = false;

  spawn_timer_ =
      nh_.createTimer(ros::Duration(3.0), &Simulator::spawnCallback, this);

  vis_obs = {
      { 0, 5, 4, 5}, { 5, 0, 5, 4 }, { 12, 0, 12, 4 }, {12, 5, 35, 5 },
      {0, 14, 24, 14 }, {25, 14, 25, 19}, { 32, 14, 32, 19 }, { 32, 14, 35, 14 }
  };

  return true;
}

void Simulator::runSimulation() {
  ros::Rate r(CONFIG.updateRate);

  while (ros::ok()) {
    if (!robot_) {
      // setup the robot
      for (Agent* agent : SCENE.getAgents()) {
        if (agent->getType() == Ped::Tagent::ROBOT) {
          robot_ = agent;
          last_robot_orientation_ =
              poseFrom2DVelocity(0, 1);
        }
      }
    }

    if (!paused_) {
      updateRobotPositionFromTF();
      SCENE.moveAllAgents();

      // After this is the degug part information.
      //paused_ = true;

      // Up to now is used for degug.
//      if(checking_frame_id <= 10){
//        publishEvaInfo();
//      }

//      else if(checking_frame_id % 5 == 0){
//        publishEvaInfo();
//      }
      //std::cout<< static_cast<int>(checking_frame_id) << std::endl;
      if(checking_frame_id % 5 == 0){
        publishEvaInfo();
        //std::cout << "frame id is : " << checking_frame_id << std::endl;
      }
//      if(checking_frame_id % 50 == 0){
//        std::cout<<"agents number:"<<SCENE.agents.size()<<std::endl;
//      }
      checking_frame_id += 1;
//      publishAgents();
//      publishGroups();
//      publishRobotPosition();
//      publishObstacles();  // TODO - no need to do this all the time.
    }

    publishAgents();
    publishGroups();
    publishRobotPosition();
    publishObstacles();  // TODO - no need to do this all the time.
    ros::spinOnce();
    r.sleep();
  }
}

void Simulator::reconfigureCB(pedsim_simulator::PedsimSimulatorConfig& config,
                              uint32_t level) {
  CONFIG.updateRate = config.update_rate;
  CONFIG.simulationFactor = config.simulation_factor;

  // update force scaling factors
  CONFIG.setObstacleForce(config.force_obstacle);
  CONFIG.setObstacleSigma(config.sigma_obstacle);
  CONFIG.setSocialForce(config.force_social);
  CONFIG.setGroupGazeForce(config.force_group_gaze);
  CONFIG.setGroupCoherenceForce(config.force_group_coherence);
  CONFIG.setGroupRepulsionForce(config.force_group_repulsion);
  CONFIG.setRandomForce(config.force_random);
  CONFIG.setAlongWallForce(config.force_wall);

  // puase or unpause the simulation
  if (paused_ != config.paused) {
    paused_ = config.paused;
  }

  ROS_INFO_STREAM("Updated sim with live config: Rate=" << CONFIG.updateRate
                                                        << " incoming rate="
                                                        << config.update_rate);
}

bool Simulator::onPauseSimulation(std_srvs::Empty::Request& request,
                                  std_srvs::Empty::Response& response) {
  paused_ = true;
  return true;
}

bool Simulator::onUnpauseSimulation(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response) {
  paused_ = false;
  return true;
}

bool Simulator::restartScenario(pedsim_srvs::ResetScene::Request &request, pedsim_srvs::ResetScene::Response &response)
{
  paused_ = true;
  SCENE.clear();
//  delete robot_;
  robot_ = NULL;
  std::cout<<"cleared"<<std::endl;
  resetScene();
  bool default_label = request.use_default_robot_position;
  std::vector<double> pos_vec;
  pos_vec.push_back(request.robot_pos[0]);
  pos_vec.push_back(request.robot_pos[1]);
  pos_vec.push_back(request.robot_pos[2]);
  resetRobotPos(default_label,pos_vec);
  paused_ = false;
  return true;
}

bool Simulator::resetRobotPos(bool label, std::vector<double> pos)
{
  pedsim_srvs::ResetRobotPos srv;
  srv.request.use_default_robot_position = label;
  srv.request.robot_pos.push_back(pos[0]);
  srv.request.robot_pos.push_back(pos[1]);
  srv.request.robot_pos.push_back(pos[2]);
  if(reset_robot_client_.call(srv))
  {
    ROS_INFO("Success");
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
  }
}

bool Simulator::resetScene()
{
  std::string map_generator_pack = ros::package::getPath("pedsim_simulator");
  std::string scene_file_param = map_generator_pack + "/scenarios/airport_activities_tong.xml";
  //std::string scene_file_param = "/home/tony-joe/river_spring/dell_ws/src/pedsim_simulator/scenarios/airport_activities_tong.xml";
  if (scene_file_param == "") {
    ROS_ERROR_STREAM("Invalid scene file: " << scene_file_param);
    return false;
  }

  ROS_INFO_STREAM("Loading scene [" << scene_file_param << "] for simulation");
  const QString scenefile = QString::fromStdString(scene_file_param);
  ScenarioReader scenario_reader;
  if (scenario_reader.readFromFile(scenefile) == false) {
    ROS_ERROR_STREAM(
        "Could not load the scene file, please check the paths and param "
        "names : "
        << scene_file_param);
    return false;
  }
  return true;
}

double Simulator::calculate_rpy_from_quat(geometry_msgs::Quaternion q)
{
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  double angle = std::atan2(siny_cosp, cosy_cosp);
  return angle;
}

void Simulator::spawnCallback(const ros::TimerEvent& event) {
  ROS_DEBUG_STREAM("Spawning new agents.");
  if(paused_){return;}
  for (const auto& sa : SCENE.getSpawnAreas()) {
    AgentCluster* agentCluster = new AgentCluster(sa->x, sa->y, sa->n);
    agentCluster->setDistribution(sa->dx, sa->dy);
    agentCluster->setType(static_cast<Ped::Tagent::AgentType>(0));
    agentCluster->setClusterId(sa->cluster_id);

    for (const auto& wp_name : sa->waypoints) {
      agentCluster->addWaypoint(SCENE.getWaypointByName(wp_name));
    }

    SCENE.addAgentCluster(agentCluster);
  }
}

void Simulator::updateRobotPositionFromTF() {
  if (!robot_) return;
  if (CONFIG.robot_mode == RobotMode::TELEOPERATION ||
      CONFIG.robot_mode == RobotMode::CONTROLLED) {
    robot_->setTeleop(true);
    robot_->setVmax(2 * CONFIG.max_robot_speed);

    // Get robot position via TF
    tf::StampedTransform tfTransform;
    try {
      transform_listener_->lookupTransform("odom", "base_footprint",
                                           ros::Time(0), tfTransform);
    } catch (tf::TransformException& e) {
      ROS_WARN_STREAM_THROTTLE(
          5.0,
          "TF lookup from base_footprint to odom failed. Reason: " << e.what());
      return;
    }
    const double x = tfTransform.getOrigin().x();
    //std::cout << last_robot_pose_.getOrigin().x() << std::endl;
    const double y = tfTransform.getOrigin().y();
    auto qf = tfTransform.getRotation();
    geometry_msgs::Quaternion q;
    tf::quaternionTFToMsg(qf,q);
    const double theta = calculate_rpy_from_quat(q);
    //std::cout << "theta is : " << theta << std::endl;

    // Here we change something a little, by using the odom as velocity rather than the truth velocity.
    const double dx = x - last_robot_pose_.getOrigin().x(),
                 dy = y - last_robot_pose_.getOrigin().y();
    const double dt =
        tfTransform.stamp_.toSec() - last_robot_pose_.stamp_.toSec();
    double vx = dx / dt, vy = dy / dt;

    if (!std::isfinite(vx)) vx = 0;
    if (!std::isfinite(vy)) vy = 0;
    ROS_DEBUG_STREAM("rx, ry: " << robot_->getx() << ", " << robot_->gety() << " vs: " << x << ", " << y);

    robot_->setX(x);
    robot_->setY(y);
    robot_->setZ(theta);
    robot_->setvx(vx);
    robot_->setvy(vy);


    ROS_DEBUG_STREAM("Robot speed: " << std::hypot(vx, vy) << " dt: " << dt);
//std::cout<<"robot position x: "<< robot_->getx()<<std::endl;
//std::cout<<"robot position y: "<< robot_->gety()<<std::endl;
    last_robot_pose_ = tfTransform;
  }
}

void Simulator::publishRobotPosition() {
  if (robot_ == nullptr) return;

  nav_msgs::Odometry robot_location;
  robot_location.header = createMsgHeader();
  robot_location.child_frame_id = "odom";

  robot_location.pose.pose.position.x = robot_->getx();
  robot_location.pose.pose.position.y = robot_->gety();
  tf::Quaternion myQuaternion;
  myQuaternion.setRPY( 0, 0, robot_->getz());
  geometry_msgs::Quaternion q;
  tf::quaternionTFToMsg(myQuaternion,q);
  robot_location.pose.pose.orientation = q;
//  if (hypot(robot_->getvx(), robot_->getvy()) < 0.05) {
//    robot_location.pose.pose.orientation = last_robot_orientation_;
//  } else {
//    robot_location.pose.pose.orientation =
//        poseFrom2DVelocity(robot_->getvx(), robot_->getvy());
//    last_robot_orientation_ = robot_location.pose.pose.orientation;
//  }
  robot_location.twist.twist.linear.x = robot_->getvx();
  robot_location.twist.twist.linear.y = robot_->getvy();

  pub_robot_position_.publish(robot_location);
}

void Simulator::publishAgents() {
  if (SCENE.getAgents().size() < 2) {
    return;
  }

  pedsim_msgs::AgentStates all_status;
  all_status.header = createMsgHeader();

  auto VecToMsg = [](const Ped::Tvector& v) {
    geometry_msgs::Vector3 gv;
    gv.x = v.x;
    gv.y = v.y;
    gv.z = v.z;
    return gv;
  };

  for (const Agent* a : SCENE.getAgents()) {
    pedsim_msgs::AgentState state;
    state.header = createMsgHeader();

    state.id = a->getId();
    state.type = a->getType();
    state.pose.position.x = a->getx();
    state.pose.position.y = a->gety();
    state.pose.position.z = a->getz();
    auto theta = std::atan2(a->getvy(), a->getvx());
    state.pose.orientation = pedsim::angleToQuaternion(theta);

    state.twist.linear.x = a->getvx();
    state.twist.linear.y = a->getvy();
    state.twist.linear.z = a->getvz();

    AgentStateMachine::AgentState sc = a->getStateMachine()->getCurrentState();
    state.social_state = agentStateToActivity(sc);
    if (a->getType() == Ped::Tagent::ELDER) {
      state.social_state = pedsim_msgs::AgentState::TYPE_STANDING;
    }

    // Skip robot.
    if (a->getType() == Ped::Tagent::ROBOT) {
      continue;
    }

    // Forces.
    pedsim_msgs::AgentForce agent_forces;
    agent_forces.desired_force = VecToMsg(a->getDesiredDirection());
    agent_forces.obstacle_force = VecToMsg(a->getObstacleForce());
    agent_forces.social_force = VecToMsg(a->getSocialForce());
    // agent_forces.group_coherence_force = a->getSocialForce();
    // agent_forces.group_gaze_force = a->getSocialForce();
    // agent_forces.group_repulsion_force = a->getSocialForce();
    // agent_forces.random_force = a->getSocialForce();

    state.forces = agent_forces;
    state.cluster_id = a->getClusterId();

    all_status.agent_states.push_back(state);
  }

  pub_agent_states_.publish(all_status);
  //std::cout << "Agent number: " << SCENE.getAgents().size() << std::endl;
}

void Simulator::publishGroups() {
  if (!CONFIG.groups_enabled) {
    ROS_DEBUG_STREAM("Groups are disabled, no group data published: flag="
                     << CONFIG.groups_enabled);
    return;
  }

  if (SCENE.getGroups().size() < 1) {
    return;
  }

  pedsim_msgs::AgentGroups sim_groups;
  sim_groups.header = createMsgHeader();

  for (const auto& ped_group : SCENE.getGroups()) {
    if (ped_group->memberCount() <= 1) continue;

    pedsim_msgs::AgentGroup group;
    group.group_id = ped_group->getId();
    group.age = 10;
    const Ped::Tvector com = ped_group->getCenterOfMass();
    group.center_of_mass.position.x = com.x;
    group.center_of_mass.position.y = com.y;

    for (const auto& member : ped_group->getMembers()) {
      group.members.emplace_back(member->getId());
    }
    sim_groups.groups.emplace_back(group);
  }
  pub_agent_groups_.publish(sim_groups);
}

//void Simulator::publishObstacles() {
//  pedsim_msgs::LineObstacles sim_obstacles;
//  sim_obstacles.header = createMsgHeader();
//  for (const auto& obstacle : SCENE.getObstacles()) {
//    pedsim_msgs::LineObstacle line_obstacle;
//    line_obstacle.start.x = obstacle->getax();
//    line_obstacle.start.y = obstacle->getay();
//    line_obstacle.start.z = 0.0;
//    line_obstacle.end.x = obstacle->getbx();
//    line_obstacle.end.y = obstacle->getby();
//    line_obstacle.end.z = 0.0;
//    sim_obstacles.obstacles.push_back(line_obstacle);
//  }
//  pub_obstacles_.publish(sim_obstacles);
//}

void Simulator::publishObstacles() {
  pedsim_msgs::LineObstacles sim_obstacles;
  sim_obstacles.header = createMsgHeader();
  for (const auto& obstacle : vis_obs) {
    pedsim_msgs::LineObstacle line_obstacle;
    line_obstacle.start.x = obstacle[0];
    line_obstacle.start.y = obstacle[1];
    line_obstacle.start.z = 0.0;
    line_obstacle.end.x = obstacle[2];
    line_obstacle.end.y = obstacle[3];
    line_obstacle.end.z = 0.0;
    sim_obstacles.obstacles.push_back(line_obstacle);
  }
  pub_obstacles_.publish(sim_obstacles);
}
void Simulator::publishEvaInfo() {
  if (SCENE.getAgents().size() < 2) {
    return;
  }

  pedsim_msgs::Eva_Info all_info;
  all_info.header = createMsgHeader();

  auto VecToMsg = [](const Ped::Tvector& v) {
    geometry_msgs::Vector3 gv;
    gv.x = v.x;
    gv.y = v.y;
    gv.z = v.z;
    return gv;
  };

  for (const Agent* a : SCENE.getAgents()) {
    pedsim_msgs::AgentState state;
    state.header = createMsgHeader();

    state.id = a->getId();
    state.type = a->getType();
    state.pose.position.x = a->getx();
    state.pose.position.y = a->gety();
    state.pose.position.z = a->getz();
    auto theta = std::atan2(a->getvy(), a->getvx());
    state.pose.orientation = pedsim::angleToQuaternion(theta);

    state.twist.linear.x = a->getvx();
    state.twist.linear.y = a->getvy();
    state.twist.linear.z = a->getvz();

    AgentStateMachine::AgentState sc = a->getStateMachine()->getCurrentState();
    state.social_state = agentStateToActivity(sc);
    if (a->getType() == Ped::Tagent::ELDER) {
      state.social_state = pedsim_msgs::AgentState::TYPE_STANDING;
    }

    // Skip robot.
    if (a->getType() == Ped::Tagent::ROBOT) {
      all_info.robot_state = state;
      continue;
    }

    // Forces.
    pedsim_msgs::AgentForce agent_forces;
    agent_forces.desired_force = VecToMsg(a->getDesiredDirection());
    agent_forces.obstacle_force = VecToMsg(a->getObstacleForce());
    agent_forces.social_force = VecToMsg(a->getSocialForce());
    // agent_forces.group_coherence_force = a->getSocialForce();
    // agent_forces.group_gaze_force = a->getSocialForce();
    // agent_forces.group_repulsion_force = a->getSocialForce();
    // agent_forces.random_force = a->getSocialForce();

    state.forces = agent_forces;

    all_info.agent_states.push_back(state);
    all_info.checking_frame_id = this->checking_frame_id;
  }

  pub_eval_info_.publish(all_info);
}


std::string Simulator::agentStateToActivity(
    const AgentStateMachine::AgentState& state) const {
  std::string activity = "Unknown";
  switch (state) {
    case AgentStateMachine::AgentState::StateWalking:
      activity = pedsim_msgs::AgentState::TYPE_INDIVIDUAL_MOVING;
      break;
    case AgentStateMachine::AgentState::StateGroupWalking:
      activity = pedsim_msgs::AgentState::TYPE_GROUP_MOVING;
      break;
    case AgentStateMachine::AgentState::StateQueueing:
      activity = pedsim_msgs::AgentState::TYPE_WAITING_IN_QUEUE;
      break;
    case AgentStateMachine::AgentState::StateShopping:
      break;
    case AgentStateMachine::AgentState::StateNone:
      break;
    case AgentStateMachine::AgentState::StateWaiting:
      break;
  }
  return activity;
}

std_msgs::Header Simulator::createMsgHeader() const {
  std_msgs::Header msg_header;
  msg_header.stamp = ros::Time::now();
  msg_header.frame_id = "odom";
  return msg_header;
}
