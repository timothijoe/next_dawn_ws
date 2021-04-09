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
#include <stdlib.h>
#include "evaluation/evaluation.h"
#include <std_msgs/Float64MultiArray.h>
//#include "evaluation/quadtree_collision_checking.h"
#include "xml_utils/tinyxml.h"

Evaluation::Evaluation(ros::NodeHandle nh){
    env_eval_info_sub = nh.subscribe("/pedsim_simulator/evaluation_info", 50, &Evaluation::data_process, this);
    entropy_pub = nh.advertise<std_msgs::Float64MultiArray>("/Frame_loss", 1);
    timer = nh.createTimer(ros::Duration(0.2), &Evaluation::timer_callback, this);
    if (COMPARED_LABEL){
		NEED_LOADING = true;
		NEED_RECORDING = false;
	}
	else{
		NEED_LOADING = false;
		NEED_RECORDING = true;
	}
	std::cout<< NEED_LOADING << std::endl;
    if(NEED_LOADING){
		load_agent_traj_from_xml();
		NEED_LOADING = false;
	}
 }


void Evaluation::data_process(const pedsim_msgs::Eva_InfoPtr& data)
{
// Update the robot position information
	std::cout << "qin zhao xiang wang" << std::endl;
	double r_x = data->robot_state.pose.position.x;
	double r_y = data->robot_state.pose.position.y;
	double r_theta = calculate_rpy_from_quat(data->robot_state.pose.orientation);
	robot_pos.change_position(r_x, r_y, r_theta);

// Update the frame information
	individual_frame_info per_frame(data->checking_frame_id);
	for(auto iter = data->agent_states.begin(); iter != data->agent_states.end(); iter++)
	{
		uint64_t person_id = (*iter).id;
		double x = (*iter).pose.position.x;
		double y = (*iter).pose.position.y;
		double angle = calculate_rpy_from_quat((*iter).pose.orientation);
		individual_agent_info individual_agent(person_id, x, y, angle);
		per_frame.multi_agents_list.push_back(individual_agent);		
	}

// Update the current frame, for collision checking
	current_frame = per_frame; 
	exe_frame_list.push_back(per_frame);
	std::cout << "Experienced "<< exe_frame_list.size()<< " frames in this experient"<<std::endl;
	int zt_check = static_cast<int>(data->checking_frame_id);
	std::cout << "Current frame id is: "<< zt_check << std::endl;
	current_frame_capacity += 1;
	if(COMPARED_LABEL){
		calc_cur_entropy(current_frame);
	}
	//calc_cur_entropy(current_frame);
	if((exe_frame_list.size()==MAX_FRAME_NUM) && NEED_RECORDING){
		write_agent_traj_to_xml();
		std::cout << "Experienced the max frames ("<< MAX_FRAME_NUM<<"), completed logging into file."<< std::endl;
		exit(0);
	}
}


void Evaluation::timer_callback(const ros::TimerEvent &){
	collision_checking();

}


void Evaluation::generate_dy_obstacles(){
// Update the obstacles according to the current frame
	Obstacles.clear();
	for(auto iter = current_frame.multi_agents_list.begin(); iter!=current_frame.multi_agents_list.end(); iter++)
	{
		double xx = (*iter).x;
		double yy = (*iter).y;
		uint64_t idx = (*iter).agent_id;
		Obstacles.push_back(Circle( idx , xx, yy));
	}
}



bool Evaluation::getAgent_fromFrame_by_id(uint64_t id, individual_agent_info& agent, individual_frame_info frame){
	//std::cout << "pedestrian number:" << multi_agents_list.size() << std::endl;
	for(auto iter = frame.multi_agents_list.begin(); iter != frame.multi_agents_list.end(); iter++){
		uint64_t p_id = (*iter).agent_id;
		if(p_id == id){
			//*agent_element =(*iter);
			agent.agent_id = (*iter).agent_id;
			agent.x = (*iter).x;
			agent.y = (*iter).y;
			agent.angle = (*iter).angle;
			return true;
		}
	}
	return false;
}


double Evaluation::calculate_rpy_from_quat(geometry_msgs::Quaternion q)
{
		double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
		double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
		double angle = std::atan2(siny_cosp, cosy_cosp);
		return angle;
}


bool Evaluation::collision_checking(){
	generate_dy_obstacles();
	double xx = robot_pos.x;
	double yy = robot_pos.y;
	Point robot_p(xx, yy);
	bool collision_check_label;
	for(size_t i = 0; i < Obstacles.size();i++){
        bool collision = Obstacles[i].collision_checking(robot_p, 0.5); 
        if (collision == true){collision_check_label = true;}
        }
    return collision_check_label;
}


void Evaluation::calc_cur_entropy(individual_frame_info current_frame_element){
	uint64_t current_frame_id = current_frame_element.frame_id;
	individual_frame_info compared_cur_frame;
	//std::cout << "compared_frame_list is :" << compared_frame_list.size()<< std::endl;
	// If two frames are alligned, which means, compared_frame_list[i].id == exe_frame_list[i].id
	// We can deirectly use compared_frame_list
	if(IS_ALLIGNED){
		bool is_alligned = compared_frame_list[exe_frame_list.size()].frame_id == current_frame_id ? true:false;
		if(!is_alligned) {IS_ALLIGNED = false;}
		else{
			compared_cur_frame = compared_frame_list[current_frame_id];
		}
	}
	if(!IS_ALLIGNED){
		uint64_t frame_list_index = 0;
		for(auto iter = compared_frame_list.begin(); iter != compared_frame_list.end(); iter++){
			uint64_t f_id = (*iter).frame_id;
			frame_list_index += 1;
			// int zt_id = static_cast<int>(f_id);
			// std::cout<< "Current frame_id is: "<< zt_id << std::endl;
			if(current_frame_id == f_id){
				compared_cur_frame = (*iter);
				break;
			}
			else if(frame_list_index == compared_frame_list.size()){
				std::cout << "not found the corresponding compared frame, disgard in this frame"<< std::endl;
				return;
			}
		}

	}	
	// Calculate the error between each frame;
	// Up to now, we have achieved the current frame info and the corresponding compared cur frame;
	// We need to calculate the error between each person.

	// First we define a current frame entropy named current_entropy
	current_frame_entropy current_entropy;
	current_entropy.difference = 0;
	current_entropy.frame_id = current_frame_id;

	// Select for each person by id in both frames, so we can calculate the error;
	uint64_t agent_list_index = 0;
	for(auto iter = current_frame_element.multi_agents_list.begin();
		iter != current_frame_element.multi_agents_list.end(); iter++){
		uint64_t c_agent_id = (*iter).agent_id;
		int zt_id = static_cast<int>(c_agent_id);
		std::cout<< "Current student_id is: "<< zt_id << std::endl;
		uint64_t ztt_id = compared_cur_frame.multi_agents_list[agent_list_index].agent_id;
		
		zt_id = static_cast<int>(ztt_id);
		std::cout<< "Compared student_id is: "<< zt_id << std::endl;
		//if(0){
		if(c_agent_id == compared_cur_frame.multi_agents_list[agent_list_index].agent_id){
			double dx = (*iter).x - compared_cur_frame.multi_agents_list[agent_list_index].x;
			double dy = (*iter).y - compared_cur_frame.multi_agents_list[agent_list_index].y;
			double ddddx = compared_cur_frame.multi_agents_list[agent_list_index].x;
			double ddddy = compared_cur_frame.multi_agents_list[agent_list_index].y;
			double a_dist = hypot(dx, dy);
			std::cout<<"Origin label: x: "<<(*iter).x <<" y: "<<(*iter).y<<std::endl;
			std::cout<<"Compared label: x: "<<ddddx<<"y: "<<ddddy<<std::endl;
			std::cout<< "Ge ming bi sheng: "<< std::endl;
			// if(c_agent_id == 1){
			// 	current_entropy.difference += a_dist;
			// }
			current_entropy.difference += a_dist;
		}
		else{
			// This is the circumstance when in each frame, the student id's order are not equal.
			individual_agent_info compared_alligend_person;
			//getAgent_fromFrame_by_id(c_agent_id, compared_alligend_person, compared_cur_frame);
			if(!getAgent_fromFrame_by_id(c_agent_id, compared_alligend_person, compared_cur_frame))
			{
				std::cout << "false to get this member" << std::endl;
				continue;
			}
			double dx = (*iter).x - compared_alligend_person.x;
			double dy = (*iter).y - compared_alligend_person.y;
			std::cout<<"Origin label: x: "<<(*iter).x <<" y: "<<(*iter).y<<std::endl;
			std::cout<<"Compared label: x: "<<compared_alligend_person.x<<"y: "<<compared_alligend_person.y<<std::endl;
			double a_dist = hypot(dx, dy);
			current_entropy.difference += a_dist;
			// if(c_agent_id == 1){
			// 	current_entropy.difference += a_dist;
			// }
			//current_entropy.difference += a_dist;
			std::cout << "one agent entropy getted."<<std::endl;
		}

	}
	agent_list_index += 1;
	total_entropy_ele.last_frame_id = current_frame_id;
	total_entropy_ele.total_difference += current_entropy.difference;
	std::cout << "Current Entropy: " << current_entropy.difference << std::endl;
	total_entropy_ele.total_frame_entropy_list.push_back(current_entropy);
	std_msgs::Float64MultiArray entropy_to_send;
	entropy_to_send.data.push_back(current_entropy.difference);
	entropy_to_send.data.push_back(current_entropy.difference);
	entropy_pub.publish(entropy_to_send);


}


void Evaluation::write_agent_traj_to_xml(){
	std::cout << "ge ming bi sheng " << std::endl;
	TiXmlDocument *writeDoc = new TiXmlDocument; // xml file pointer
	//file format declaration
	TiXmlDeclaration *decl = new TiXmlDeclaration("1.0", "UTF-8", "yes");
	writeDoc->LinkEndChild(decl); // writing content into file
	int n = exe_frame_list.size();
	TiXmlElement *RootElement = new TiXmlElement("Total_Frames");//根元素
	RootElement->SetAttribute("num", n); //属性
	writeDoc->LinkEndChild(RootElement);
	for(int i=0; i<n; i++)//n个父节点
	{
		TiXmlElement *FrameElement = new TiXmlElement("Frame");//Stu
		//设置属性
		FrameElement->SetAttribute("frame_id", exe_frame_list[i].frame_id);
		RootElement->LinkEndChild(FrameElement);//父节点写入文档

		for(size_t j = 0; j < exe_frame_list[i].multi_agents_list.size(); j++)
		{
			TiXmlElement *agentElement = new TiXmlElement("Agent");
			agentElement->SetAttribute("agent_id", exe_frame_list[i].multi_agents_list[j].agent_id);
			FrameElement->LinkEndChild(agentElement);

			// write x information into the file
			TiXmlElement *x_posElement = new TiXmlElement("x");
			agentElement->LinkEndChild(x_posElement);
			string x_pose_string = to_string(exe_frame_list[i].multi_agents_list[j].x);
			const char* xx = x_pose_string.c_str();
			TiXmlText *x_posContent = new TiXmlText(xx);
			x_posElement->LinkEndChild(x_posContent);

			// write x information into the file
			TiXmlElement *y_posElement = new TiXmlElement("y");
			agentElement->LinkEndChild(y_posElement);
			string y_pose_string = to_string(exe_frame_list[i].multi_agents_list[j].y);
			const char* yy = y_pose_string.c_str();
			TiXmlText *y_posContent = new TiXmlText(yy);
			y_posElement->LinkEndChild(y_posContent);

						// write x information into the file
			TiXmlElement *r_posElement = new TiXmlElement("angle");
			agentElement->LinkEndChild(r_posElement);
			string r_pose_string = to_string(exe_frame_list[i].multi_agents_list[j].angle);
			const char* rr = r_pose_string.c_str();
			TiXmlText *r_posContent = new TiXmlText(rr);
			r_posElement->LinkEndChild(r_posContent);
		}
	}
	
	writeDoc->SaveFile("/home/tony-joe/agent_simu_ws/src/pedsim_ros/evaluation/xml_files/version1.xml");
	delete writeDoc;
std::cout << "writing done " << std::endl;
}


void Evaluation::load_agent_traj_from_xml(){
	TiXmlDocument mydoc("/home/tony-joe/agent_simu_ws/src/pedsim_ros/evaluation/xml_files/version1.xml");//xml文档对象
	bool loadOk=mydoc.LoadFile();//加载文档
	if(!loadOk)
	{
		cout<<"could not load the test file.Error:"<<mydoc.ErrorDesc()<<endl;
		exit(1);
	}
	compared_frame_list.clear();
	TiXmlElement *RootElement = mydoc.RootElement();
	cout<< "[root name]" << RootElement->Value() <<"\n";
	string frame_num_str = RootElement->FirstAttribute()->Value();
	compared_frame_capacity = static_cast<uint64_t>(std::stoi(frame_num_str));
	TiXmlElement *pEle=RootElement;
		//遍历该结点
	for(TiXmlElement *FrameElement = pEle->FirstChildElement();//第一个子元素
		FrameElement != NULL;
		FrameElement = FrameElement->NextSiblingElement())//下一个兄弟元素
	{
		//std::cout<< "Ge ming bi sheng"<<std::endl;
		string frame_id_str = FrameElement->FirstAttribute()->Value();
		uint64_t frame_id = static_cast<uint64_t>(std::stoi(frame_id_str));
		std::cout << "frame_id: "<< frame_id << std::endl;
		individual_frame_info per_frame(frame_id);
		//std::cout<< "Ge ming bi sheng1"<<std::endl;
		for(TiXmlElement *AgentElement = FrameElement->FirstChildElement();//第一个子元素
		AgentElement != NULL;
		AgentElement = AgentElement->NextSiblingElement())
		{
			//std::cout<< "Ge ming bi sheng2"<<std::endl;
			string agent_id_str = AgentElement->FirstAttribute()->Value();
			uint64_t agent_id = static_cast<uint64_t>(std::stoi(agent_id_str));
			TiXmlElement *pAttr=AgentElement->FirstChildElement();
			string x_str = pAttr->FirstChild()->Value();
			// std::cout<< x_str<<std::endl;
		    double xx = atof(x_str.c_str());
			// std::cout<< "Ge ming bi sheng4"<<std::endl;
			pAttr=pAttr->NextSiblingElement();
			//pAttr=pAttr->Next();
			string y_str = pAttr->FirstChild()->Value();
			// std::cout<< y_str<<std::endl;
			double yy = atof(y_str.c_str());
			pAttr=pAttr->NextSiblingElement();
			string r_str = pAttr->FirstChild()->Value();
			double rr = atof(r_str.c_str());
			// std::cout<< "Ge ming bi sheng3"<<std::endl;
			individual_agent_info per_agent(agent_id, xx, yy, rr);
			per_frame.multi_agents_list.push_back(per_agent);
			std::cout << "x = : " << xx << " and y =: "<< yy <<std::endl;
			//per_frame.multi_agents_list.clear();
		}
		compared_frame_list.push_back(per_frame);
	}
	std::cout<< "Success Loading! "<<std::endl;
}






int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_checking");
    const ros::NodeHandle& nh = ros::NodeHandle("~");
    Evaluation node(nh);
    ros::Duration(1).sleep();
    ros::spin();
    return 0;
}

