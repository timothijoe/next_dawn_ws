#include "evaluation/compare_frame.h"

compareFrame::compareFrame(ros::NodeHandle nh)
{
  env_eval_info_sub = nh.subscribe("/pedsim_simulator/evaluation_info", 1, &compareFrame::data_process, this);
  //timer = nh.createTimer(ros::Duration(0.2), &compareFrame::timer_callback, this);

}

void compareFrame::data_process(const pedsim_msgs::Eva_InfoPtr &data)
{
  // Update the robot position information
    double r_x = data->robot_state.pose.position.x;
    double r_y = data->robot_state.pose.position.y;
    double r_theta = calculate_rpy_from_quat(data->robot_state.pose.orientation);
    _robot_pos << r_x, r_y, r_theta;

  // Update the frame information
    current_frame.frame_id = data->checking_frame_id;
    current_frame.agents_list.clear();
    for(auto iter = data->agent_states.begin(); iter != data->agent_states.end(); iter++)
    {
      int person_id = (*iter).id;
      double x = (*iter).pose.position.x;
      double y = (*iter).pose.position.y;
      //double angle = calculate_rpy_from_quat((*iter).pose.orientation);
      single_ped_xvi individual_agent = single_ped_xvi(person_id, x, y);
      current_frame.agents_list.push_back(individual_agent);
    }
  // Update the current frame, for collision checking
    cur_frame_capacity += 1;
    // if(exe_frame_list.size()==101){
    // 	write_agent_traj_to_xml();
    // }
    calc_cur_entropy(current_frame);
}

double compareFrame::calculate_rpy_from_quat(geometry_msgs::Quaternion q)
{
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  double angle = std::atan2(siny_cosp, cosy_cosp);
  return angle;
}

void compareFrame::calc_cur_entropy(singleFrame current_frame_)
{

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_checking");
    const ros::NodeHandle& nh = ros::NodeHandle("~");
    compareFrame node(nh);
    ros::Duration(1).sleep();
    ros::spin();
    return 0;
}
