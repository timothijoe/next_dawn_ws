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
#include "evaluation/collision_checker.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_checking");
    const ros::NodeHandle& nh = ros::NodeHandle("~");
    CollisionChecker collision_checker(nh);
    std::cout << "hhh "<<std::endl;
    ros::spin();
    return 0;
}
