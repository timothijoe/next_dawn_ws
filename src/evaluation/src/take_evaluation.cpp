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
#include "evaluation/take_evaluation.h"
#include "xml_utils/tinyxml.h"

Evaluation::Evaluation(ros::NodeHandle nh){
    env_eval_info_sub = nh.subscribe("/pedsim_simulator/evaluation_info", 1, &Evaluation::data_process, this);
    timer = nh.createTimer(ros::Duration(0.2), &Evaluation::timer_callback, this);
 }

bool Evaluation::getAgent_fromFrame_by_id(uint64_t id, struct individual_agent_info agent, struct individual_frame_info frame){
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

void Evaluation::data_process(const pedsim_msgs::Eva_InfoPtr& data)
{
// Update the robot position information
	double r_x = data->robot_state.pose.position.x;
	double r_y = data->robot_state.pose.position.y;
	double r_theta = calculate_rpy_from_quat(data->robot_state.pose.orientation);
	robot_pos.change_position(r_x, r_y, r_theta);

// Update the frame information
	struct individual_frame_info per_frame(data->checking_frame_id);
	for(auto iter = data->agent_states.begin(); iter != data->agent_states.end(); iter++)
	{
		uint64_t person_id = (*iter).id;
		double x = (*iter).pose.position.x;
		double y = (*iter).pose.position.y;
		double angle = calculate_rpy_from_quat((*iter).pose.orientation);
		struct individual_agent_info individual_agent(person_id, x, y, angle);
		per_frame.multi_agents_list.push_back(individual_agent);		
	}
// Update the current frame, for collision checking
	current_frame = per_frame; 
	exe_frame_list.push_back(per_frame);
	std::cout << exe_frame_list.size()<<std::endl;
	int zt_check = static_cast<int>(data->checking_frame_id);
	std::cout << zt_check << std::endl;
	current_frame_capacity += 1;
	// if(exe_frame_list.size()==101){
	// 	write_agent_traj_to_xml();
	// }
	calc_cur_entropy(current_frame);

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
	
	writeDoc->SaveFile("/home/tony-joe/ped_simu_ws/src/pedsim_ros/pedsim_simulator/xml_files/try_writing.xml");
	delete writeDoc;
std::cout << "writing done " << std::endl;
}


void Evaluation::load_agent_traj_from_xml(){
	TiXmlDocument mydoc("/home/tony-joe/ped_simu_ws/src/pedsim_ros/pedsim_simulator/xml_files/try_writing.xml");//xml文档对象
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
		struct individual_frame_info per_frame(frame_id);
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
			struct individual_agent_info per_agent(agent_id, xx, yy, rr);
			per_frame.multi_agents_list.push_back(per_agent);
			std::cout << "x = : " << xx << " and y =: "<< yy <<std::endl;
			//per_frame.multi_agents_list.clear();
		}
		compared_frame_list.push_back(per_frame);
	}
	std::cout<< "Success Loading! "<<std::endl;
}


void Evaluation::calc_cur_entropy(struct individual_frame_info current_frame_element){
	uint64_t current_frame_id = current_frame_element.frame_id;
	struct individual_frame_info compared_cur_frame;
	std::cout << "compared_frame_list is :" << compared_frame_list.size()<< std::endl;
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
	// Calculate the error between each frame;
	struct current_frame_entropy current_entropy;
	current_entropy.difference = 0;
	current_entropy.frame_id = current_frame_id;
	uint64_t agent_list_index = 0;
	for(auto iter = current_frame_element.multi_agents_list.begin();
		iter != current_frame_element.multi_agents_list.end(); iter++){
		uint64_t c_agent_id = (*iter).agent_id;
		int zt_id = static_cast<int>(c_agent_id);
		std::cout<< "Current student_id is: "<< zt_id << std::endl;
		uint64_t ztt_id = compared_cur_frame.multi_agents_list[agent_list_index].agent_id;
		zt_id = static_cast<int>(ztt_id);
		std::cout<< "Compared student_id is: "<< zt_id << std::endl;
		if(c_agent_id == compared_cur_frame.multi_agents_list[agent_list_index].agent_id){
			double dx = (*iter).x - compared_cur_frame.multi_agents_list[agent_list_index].x;
			double dy = (*iter).y - compared_cur_frame.multi_agents_list[agent_list_index].y;
			double a_dist = hypot(dx, dy);
			std::cout<< "Ge ming bi sheng: "<< std::endl;
			current_entropy.difference += a_dist;
		}






		// int zt_id = static_cast<int>(c_agent_id);
		// std::cout<< "Current student_id is: "<< zt_id << std::endl;
		// for(auto fter = compared_cur_frame.multi_agents_list.begin();
		// 	iter != compared_cur_frame.multi_agents_list.end(); iter++){

		// }







		// struct individual_agent_info this_agent();
		// if(getAgent_fromFrame_by_id(c_agent_id, this_agent,compared_cur_frame)){
		// 	double dx = (*iter).x - this_agent.x;
		// 	double dy = (*iter).y - this_agent.y;
		// 	std::cout << "dx: "<< dx <<std::endl;
		// 	double a_dist = hypot(dx, dy);
		// 	current_entropy.difference += a_dist;
		// }
	}
	total_entropy_ele.last_frame_id = current_frame_id;
	total_entropy_ele.total_difference += current_entropy.difference;
	std::cout << "Current Entropy: " << current_entropy.difference << std::endl;
	total_entropy_ele.total_frame_entropy_list.push_back(current_entropy);


}


void Evaluation::timer_callback(const ros::TimerEvent &){
	collision_checking();
	if(NEED_LOADING){
		load_agent_traj_from_xml();
		NEED_LOADING = false;
     //std::cout<<"timer ok"<<std::endl;
	}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_checking");
    const ros::NodeHandle& nh = ros::NodeHandle("~");
    Evaluation node(nh);
    ros::Duration(1).sleep();
    ros::spin();
    return 0;
}

