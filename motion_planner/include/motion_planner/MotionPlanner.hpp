/**
 * @file MotionPlanner.hpp
 * @author Ali Aydın Küçükçöllü (kucukcollu@outlook.com)
 * @brief OMPL based 2D RRT* planner for motion planning in ROS.
 * @version 0.1
 * @date 2022-03-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __MOTION_PLANNER_HPP__
#define __MOTION_PLANNER_HPP__

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

// COSTOM SERVICE
#include "motion_planner/PlanningService.h"

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>

// for smart_pointers
#include <mutex>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace planner
{
    class MotionPlanner
    {
        private:
            ros::NodeHandle *nh_;
            
            ros::Publisher path_pub_;       // to pusblish planned path in nav_msgs::Path format
            ros::Publisher start_pub_;      // to publish start pose in visualization_msgs::Marker format
            ros::Publisher goal_pub_;       // to publish goal pose in visualization_msgs::Marker format
            ros::Subscriber marker_sub_;    // to subscribe obstacles in visualization_msgs::MarkerArray format

           /**
            * @brief Planning service callback
            * @input start(X,Y) and goal(X,Y) points
            * @output is path planned or not and calculation time
            */
            ros::ServiceServer planning_server_;

            nav_msgs::Path planned_path_;               // publish planned path
            visualization_msgs::Marker start_point_;    // visualize start point
            visualization_msgs::Marker goal_point_;     // visualize goal point
            visualization_msgs::Marker obstacle_;       // single obstacle
            visualization_msgs::MarkerArray obstacles_; // all obstacles in the simulation environment

            std_msgs::Float64 start_x_;                // start point x coordinate
            std_msgs::Float64 start_y_;                // start point y coordinate
            std_msgs::Float64 goal_x_;                // goal point x coordinate
            std_msgs::Float64 goal_y_;               // goal point y coordinate

            double inflation_;                       // inflation radius for the obstacles
            
            /**
             * @brief Converts OMPL path to ROS path
             * 
             * @param pdef 
             * @return nav_msgs::Path 
             */
            nav_msgs::Path convert_path(ob::ProblemDefinition* pdef);

        public:
            /**
             * @brief Construct a new Motion Planner object
             * 
             * @param nh 
             */
            MotionPlanner(ros::NodeHandle *nh);

            /**
             * @brief Destructor
             * 
             */
            ~MotionPlanner();

            /**
             * @brief Visualize start point
             * 
             * @return visualization_msgs::Marker 
             */
            visualization_msgs::Marker start_point_publisher();

            /**
             * @brief Visualize goal point
             * 
             * @return visualization_msgs::Marker 
             */
            visualization_msgs::Marker goal_point_publisher();

            /**
             * @brief Obstacle subscriber callback function
             * 
             * @param msg 
             */
            void obstacle_callback(const visualization_msgs::MarkerArray::ConstPtr& msg);

            /**
             * @brief Service server callback function: main node function.
             * It sets paramters, calls planner and publishes path.
             * 
             * @param req 
             * @param res 
             * @return bool
             */
            bool get_points(motion_planner::PlanningService::Request &req,motion_planner::PlanningService::Response &res);

            /**
             * @brief Obstacle avoidance function. It calculates
             * the distance between the robot and the obstacle for every
             * single obstacle in the environment for current state.
             * 
             * @param state 
             * @return true 
             * @return false 
             */
            bool is_valid(const ob::State* state);
    };
}

#endif // __MOTION_PLANNER_HPP__