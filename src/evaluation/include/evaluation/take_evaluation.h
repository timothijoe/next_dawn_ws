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
struct individual_agent_info{
	uint64_t agent_id;
	double x;
	double y;
	double angle; // In rad
	individual_agent_info(uint64_t id, double xx, double yy, double angle = 0):
	agent_id{id}, x{xx}, y{yy}, angle{angle} {}
	// individual_agent_info(uint64_t id):
	// agent_id{id}, x{0}, y{0}, angle{0} {}
};


// Basic robot information, used for collision checking on real time
struct robot_position_info{
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
struct individual_frame_info{
	uint64_t frame_id;
	std::vector<individual_agent_info> multi_agents_list;
	individual_frame_info(uint64_t id=0):
	frame_id{id} {}
	//bool get_agent_by_id(uint64_t id, struct individual_agent_info agent_element);

};
// bool individual_frame_info::get_agent_by_id(uint64_t id, struct individual_agent_info agent_element){
// 	//std::cout << "pedestrian number:" << multi_agents_list.size() << std::endl;
// 	for(auto iter = multi_agents_list.begin(); iter != multi_agents_list.end(); iter++){
// 		uint64_t p_id = (*iter).agent_id;
// 		if(p_id == id){
// 			//*agent_element =(*iter);
// 			agent_element.agent_id = (*iter).agent_id;
// 			agent_element.x = (*iter).x;
// 			agent_element.y = (*iter).y;
// 			agent_element.angle = (*iter).angle;
// 			return true;
// 		}
// 	}
// 	return false;
// }

struct current_frame_entropy{
	uint64_t frame_id;
	double difference;
};

struct total_entropy_elements{
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
	struct total_entropy_elements total_entropy_ele;
	// std::vector<double> total_frames_entropy;
	bool NEED_LOADING = true;
	bool ALREADY_LOADED = false;
	uint64_t current_frame_capacity = 0; // current frame numbers of exe_frame_list; will increase with time
	uint64_t compared_frame_capacity; // total frame numbers that we load from file, which is the maximum 
	// number that we can compare



// Robot infomation
	struct robot_position_info robot_pos;

// Current frame pedestrian, used for collision checking
	struct individual_frame_info current_frame;
// Get the pedestrians position for collision checking
	std::vector<Circle> Obstacles;
	//Line obstacle(0,0);

// subscribers and publishers, 
    ros::Subscriber env_eval_info_sub;  // pedestrian subscriber
    ros::Timer timer;

public:
    Evaluation(ros::NodeHandle nh);
    void data_process(const pedsim_msgs::Eva_InfoPtr& data);
    void timer_callback(const ros::TimerEvent&);
    void generate_dy_obstacles(); // generate dynamic obstacles
    bool collision_checking();
    double calculate_rpy_from_quat(geometry_msgs::Quaternion q);
    void write_agent_traj_to_xml();
    void load_agent_traj_from_xml();
    void calc_cur_entropy(struct individual_frame_info current_frame_element);
    bool getAgent_fromFrame_by_id(uint64_t id, struct individual_agent_info agent, struct individual_frame_info frame);
};
#endif
