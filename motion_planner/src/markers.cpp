/**
 * @file markers.cpp
 * @author Ali Aydın Küçükçöllü (kucukcollu@outlook.com)
 * @brief Obstacle node for the motion_planner package.
 * @version 0.1
 * @date 2022-03-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <stdlib.h>
#include <random>
#include <sstream>

// maximum number of obstacles in the map
#define MAX_OBSTACLE 100

/**
 * @brief Create marker array of  obstacles
 * 
 * @return visualization_msgs::MarkerArray 
 */
visualization_msgs::MarkerArray create_obstacles()
{
    int count = 0; // obstacle counter
    visualization_msgs::MarkerArray obstacle_vector;
    
    while(true)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        marker.type = visualization_msgs::Marker::CYLINDER;

        marker.action = visualization_msgs::Marker::ADD;
        
        // set random positions for X and Y axis
        std::random_device rseed1;
        std::mt19937 rng1(rseed1());
        std::uniform_int_distribution<int> dist1(-50,50);

        std::random_device rseed2;
        std::mt19937 rng2(rseed2());
        std::uniform_int_distribution<int> dist2(-50,50);
        
        marker.pose.position.x =  dist1(rng1);
        marker.pose.position.y =  dist2(rng2);
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        // set random size for the obstacle
        marker.scale.x = rand() % 4 + 1;
        marker.scale.y = marker.scale.x ;
        marker.scale.z = 1.0;

        // Set the color
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        // to be set difffrent namespaces for every
        // single obstacle
        std::string s = std::to_string(count);
        marker.ns = "obstacles"+s;

        marker.id = count;
        
        // add markers to the vector
        obstacle_vector.markers.push_back(marker);

        count ++;

        // when maximum number of obstacles is reached
        if(count == MAX_OBSTACLE)
            break;

    }

    // return obstacles
    return obstacle_vector;
}

int main(int argc,char **argv)
{
    // initialize the ROS node
    ros::init(argc,argv,"markers");
    ros::NodeHandle nh;

    // create a publisher for obstacles
    ros::Publisher obstacle_publisher = nh.advertise<visualization_msgs::MarkerArray>("markers",100);

    visualization_msgs::MarkerArray obstacles;

    // call the obstacle function
    obstacles = create_obstacles();

    while(ros::ok())
    {
        // publish the obstacles continuously
        obstacle_publisher.publish(obstacles);
        ros::spinOnce();
    }

    return 0;
}