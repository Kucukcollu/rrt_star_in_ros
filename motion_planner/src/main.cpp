/**
 * @file main.cpp
 * @author Ali Aydın Küçükçöllü (kucukcollu@outlook.com)
 * @brief OMPL based 2D RRT* planner for motion planning in ROS.
 * @version 0.1
 * @date 2022-03-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "motion_planner/MotionPlanner.hpp"

int main(int argc, char** argv)
{
    // initialize ROS node
    ros::init(argc, argv, "motion_planner");
    ros::NodeHandle nh;

    // create MotionPlanner object
    planner::MotionPlanner planner(&nh);
    ROS_INFO("Motion planner initialized...");
    
    ros::spin();
    return 0;
}