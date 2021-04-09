#ifndef _evaluation_h_
#define _evaluation_h_
#include <ros/console.h>
#include <ros/ros.h>
#include <functional>
#include <iostream>
#include <pedsim_msgs/AgentState.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/Eva_Info.h>
//#include "pedsim_simulator/computation_geometry_utils.h"
#include <std_srvs/Empty.h>
#include <math.h>
#include <bits/stdc++.h>

using namespace std;


using namespace util_ics_env;
// Basic individual information, which should be contained in one frame
class individual_agent_info{
public:
	uint64_t agent_id;
	double x;
	double y;
	double angle; // In rad
	individual_agent_info(uint64_t id = 0, double xx = 0, double yy = 0, double angle = 0):
	agent_id{id}, x{xx}, y{yy}, angle{angle} {}
};


// Basic robot information, used for collision checking on real time
class robot_position_info{
public:
	double x;
	double y;
	double angle; // In rad
	robot_position_info(double xx = 0, double yy = 0, double angle = 0):
	x{xx}, y{yy}, angle{angle} {}

	void change_position(double xx, double yy, double tangle);
};
void robot_position_info::change_position(double xx, double yy, double tangle)
{
		x = xx;
		y = yy;
		angle = tangle;
}

// Individual frame, we have vector to contain all the frames
class individual_frame_info{
public:
	uint64_t frame_id;
	std::vector<individual_agent_info> multi_agents_list;
	individual_frame_info(uint64_t id=0):
	frame_id{id} {}
};

// Three parts above are used for storing the information we receive



// The two class below are for calculate and store entropy
class current_frame_entropy{
public:
	uint64_t frame_id;
	double difference;
};


class total_entropy_elements{
public:
	uint64_t last_frame_id;
	double total_difference = 0;
	std::vector<current_frame_entropy> total_frame_entropy_list;
};

// Main class, we use it in three basic functions:
// Collision Checking;
// Calculating time execution time 
// Record the poisition and calculate the entropy.

// It will send command to the simulator by means of service.
class Evaluation
{

private:
// Two vectors, one , which named compared, used for the one which not affected by the robot, need to load 
// from file;  the other, which named execution, used for the real time compare to get the current entropy.
	std::vector<individual_frame_info> compared_frame_list;
	std::vector<individual_frame_info> exe_frame_list;
	total_entropy_elements total_entropy_ele;
	// std::vector<double> total_frames_entropy;
	bool COMPARED_LABEL = true;
	bool NEED_LOADING ;//= true;
	bool NEED_RECORDING; // = false;
	bool IS_ALLIGNED = false;

	//bool ALREADY_LOADED = false;
	size_t MAX_FRAME_NUM = 250;//0; // 1(10) min x 60s x 5 times = 3000 times
	//size_t MAX_FRAME_NUM = 1200;//0; // 1(10) min x 60s x 5 times = 3000 times
	uint64_t current_frame_capacity = 0; // current frame numbers of exe_frame_list; will increase with time
	uint64_t compared_frame_capacity; // total frame numbers that we load from file, which is the maximum 
	// number that we can compare



// Robot infomation
	robot_position_info robot_pos;

// Current frame pedestrian, used for collision checking
	individual_frame_info current_frame;
// Get the pedestrians position for collision checking, get from current_frame
	std::vector<Circle> Obstacles;

// subscribers and publishers, 
    ros::Subscriber env_eval_info_sub;  // pedestrian subscriber
    ros::Publisher entropy_pub; // publish the next state info to other nodes
    ros::Timer timer;

public:
    Evaluation(ros::NodeHandle nh);
    void data_process(const pedsim_msgs::Eva_InfoPtr& data);
    void timer_callback(const ros::TimerEvent&);
    void generate_dy_obstacles(); // generate dynamic obstacles
    bool collision_checking();
    void calc_cur_entropy(individual_frame_info current_frame_element);
    bool getAgent_fromFrame_by_id(uint64_t id, individual_agent_info& agent, individual_frame_info frame);
    double calculate_rpy_from_quat(geometry_msgs::Quaternion q);
    void write_agent_traj_to_xml();
    void load_agent_traj_from_xml();
};
#endif
