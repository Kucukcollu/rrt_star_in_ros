/**
 * @file MotionPlanner.cpp
 * @author Ali Aydın Küçükçöllü (kucukcollu@outlook.com)
 * @brief OMPL based 2D RRT* planner for motion planning in ROS.
 * @version 0.1
 * @date 2022-03-022
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "motion_planner/MotionPlanner.hpp"

/**
 * @brief Set the publishers and subscribers for the related topics.
 * And call the main Service Server.
 * 
 * @param nh 
 */
planner::MotionPlanner::MotionPlanner(ros::NodeHandle *nh)
{

    this->nh_ = nh;                                                                                                             // set node handle

    path_pub_ = nh_->advertise<nav_msgs::Path>("/planned_path",2);                                                               // Path publisher
    start_pub_ = nh_->advertise<visualization_msgs::Marker>("/start_pose",2);                                                    // Start point publisher
    goal_pub_ = nh_->advertise<visualization_msgs::Marker>("/goal_pose",2);                                                      // Goal point publisher
    marker_sub_ = nh_->subscribe<visualization_msgs::MarkerArray>("/markers",2,&planner::MotionPlanner::obstacle_callback,this); // Obstacle subscriber
    planning_server_ = nh_->advertiseService("/planning_service",&planner::MotionPlanner::get_points,this);                      // Planning Service

    inflation_ = 1.0;                                                                                                            // set inflation value as 1.0
    
    ros::Duration(0.5).sleep();

}

/**
 * @brief Destructor
 * 
 */
planner::MotionPlanner::~MotionPlanner()
{
}

/**
 * @brief Visualize start point to prevent conflict in RViz
 * 
 * @return visualization_msgs::Marker 
 */
visualization_msgs::Marker planner::MotionPlanner::start_point_publisher()
{
    visualization_msgs::Marker start_point;
    start_point.header.frame_id = "/map";
    start_point.header.stamp = ros::Time::now();

    start_point.type = visualization_msgs::Marker::SPHERE;

    start_point.action = visualization_msgs::Marker::ADD;


    start_point.pose.position.x =  start_x_.data;
    start_point.pose.position.y =  start_y_.data;
    start_point.pose.position.z = 0.0;
    start_point.pose.orientation.x = 0.0;
    start_point.pose.orientation.y = 0.0;
    start_point.pose.orientation.z = 0.0;
    start_point.pose.orientation.w = 1.0;

    // Set the scale of the start_point_ -- 1x1x1 here means 1m on a side
    start_point.scale.x = 2;
    start_point.scale.y = 2;
    start_point.scale.z = 2;

    // Set the color -- be sure to set alpha to something non-zero!
    start_point.color.r = 0.0f;
    start_point.color.g = 0.0f;
    start_point.color.b = 1.0f;
    start_point.color.a = 1.0;

    start_point.ns = "start";

    start_point.id = 0;
    
    //start_pub_.publish(start_point_);
    return start_point;

}

/**
 * @brief Visualize goal point to prevent conflict in RViz
 * 
 * @return visualization_msgs::Marker 
 */
visualization_msgs::Marker planner::MotionPlanner::goal_point_publisher()
{
    visualization_msgs::Marker goal_point;
    goal_point.header.frame_id = "/map";
    goal_point.header.stamp = ros::Time::now();

    goal_point.type = visualization_msgs::Marker::SPHERE;

    goal_point.action = visualization_msgs::Marker::ADD;


    goal_point.pose.position.x =  goal_x_.data;
    goal_point.pose.position.y =  goal_y_.data;
    goal_point.pose.position.z = 0.0;
    goal_point.pose.orientation.x = 0.0;
    goal_point.pose.orientation.y = 0.0;
    goal_point.pose.orientation.z = 0.0;
    goal_point.pose.orientation.w = 1.0;

    // Set the scale of the goal_point_ -- 1x1x1 here means 1m on a side
    goal_point.scale.x = 2;
    goal_point.scale.y = 2;
    goal_point.scale.z = 2;

    // Set the color -- be sure to set alpha to something non-zero!
    goal_point.color.r = 1.0f;
    goal_point.color.g = 0.0f;
    goal_point.color.b = 0.0f;
    goal_point.color.a = 1.0;
    
    //goal_pub_.publish(goal_point_);
    return goal_point;
}

/**
 * @brief Obstacle subscriber callback function.
 * Get the obstacles.
 * 
 * @param msg 
 */
void planner::MotionPlanner::obstacle_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    obstacles_ = *msg;
}

/**
 * @brief Service server callback function: main node function.
 * It sets paramters, calls planner and publishes path.
 * 
 * @param req 
 * @param res 
 * @return bool
 */
bool planner::MotionPlanner::get_points(motion_planner::PlanningService::Request &req,motion_planner::PlanningService::Response &res)
{
    // read the user defined start-goal points
    start_x_.data = req.start_point_x;
    start_y_.data = req.start_point_y;
    goal_x_.data = req.goal_point_x;
    goal_y_.data = req.goal_point_y;

    // check if the start and goal points are in the map
    if(start_x_.data > 50 || start_x_.data < -50 || goal_y_.data > 50 || goal_y_.data < -50)
    {
        ROS_ERROR("Selected points are out of the map");
        res.is_path_valid = false;
        res.calculation_time = ros::Time(0);
        return true;
    }

    ros::Duration(1.0).sleep();
    
    ROS_INFO("Planning...");
    
    /// bounds for the x axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordXBound;

    /// bounds for the y axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordYBound;

    /// create space
    std::shared_ptr<ompl::base::StateSpace> space;

    // create bounds for the x axis
    coordXBound.reset(new ob::RealVectorBounds(1));
    coordXBound->setLow(-50.0);
    coordXBound->setHigh(50.0);

    // create bounds for the y axis
    coordYBound.reset(new ob::RealVectorBounds(1));
    coordYBound->setLow(-50.0);
    coordYBound->setHigh(50.0);

    // construct the state space we are planning in
    auto coordX(std::make_shared<ob::RealVectorStateSpace>(1));
    auto coordY(std::make_shared<ob::RealVectorStateSpace>(1));
    space = coordX +coordY;

    // set bounds for the x axis
    coordX->setBounds(*coordXBound.get());

    // set bounds for the y axis
    coordY->setBounds(*coordYBound.get());

    // This contains all the information about the space planning is done in
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    // set state validity checking for this space
    // for obstacle avoidance behavior
    ROS_INFO("Validity checker searching...");
    si->setStateValidityChecker(boost::bind(&planner::MotionPlanner::is_valid, this, _1));
    si->setup();

    ROS_INFO("Setting starting point...");
    std::shared_ptr<ob::ScopedState<>> start;
    start.reset(new ob::ScopedState<>(space));
    (*start.get())[0]=start_x_.data;
    (*start.get())[1]=start_y_.data;

    ROS_INFO("Setting up the problem...");
    std::shared_ptr<ob::ScopedState<>> goal;
    goal.reset(new ob::ScopedState<>(space));
    (*goal.get())[0]=goal_x_.data;
    (*goal.get())[1]=goal_y_.data;

    // define the problem that we are going to solve
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    ROS_INFO("Setting up the problem definition...");
    pdef->setStartAndGoalStates(start->get(), goal->get());

    // create the planner as RRT*
    ROS_INFO("Setting up the planner as RRT STAR...");
    auto planner(std::make_shared<og::RRTstar>(si));
    
    planner->setProblemDefinition(pdef);
    //planner->setRange(5.0);                 // MAX step length.  This is the maximum distance that the motion planner will move on this call
    planner->setGoalBias(0.1);             // goal bias: between 0.0 and 1.0. Default: 0.05
    planner->setDelayCC(true);              // validity checker time delay
    planner->setup();


    double start_time = ros::Time::now().toSec();
    
    // solve the pre-defined problem
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);
    
    // if the problem is solved
    if (solved)
    {
        double end_time = ros::Time::now().toSec();
        double calc_time = end_time - start_time;     // calculation time

        ROS_INFO("Path solution found!");
        res.is_path_valid = true;
        res.calculation_time = ros::Time(calc_time);

        planned_path_ = convert_path(pdef.get());

        path_pub_.publish(planned_path_);
        start_point_ = start_point_publisher();
        start_pub_.publish(start_point_);
        goal_point_ = goal_point_publisher();
        goal_pub_.publish(goal_point_);

        return true;
    }
    // problem could not solved for a reason
    else
    {
        ROS_ERROR("No path solution found!");
        res.is_path_valid = false;
        res.calculation_time = ros::Time(0);
        return true;
    }
}

/**
 * @brief Obstacle avoidance function.
 * It calculates and controls every state of the way.
 * It is also possible to adjust inflation move away from the obstacles.
 * 
 * @param state 
 * @return bool
 */
bool planner::MotionPlanner::is_valid(const ompl::base::State* state)
{
    // get the X and Y coordinates of the state
    const auto *coordX = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *coordY = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

    // check the every single obstacles in the map
    // to avoid the path to be in collision.
    for(int i = 0; i < obstacles_.markers.size(); i++)
    {
        double x_obs = obstacles_.markers[i].pose.position.x;
        double y_obs = obstacles_.markers[i].pose.position.y;
        double radius = obstacles_.markers[i].scale.x;

        if(sqrt(pow(coordX->values[0]-x_obs,2)+pow(coordY->values[0]-y_obs,2)) - (radius+inflation_)/2 < 0)
        {
            return false;
        }

        // if the given goal is in collision with the obstacle
        if(start_x_.data == x_obs && start_y_.data == y_obs)
        {
            ROS_ERROR("Start point is in collision with the obstacle!");
            return false;
        }

        if(goal_x_.data == x_obs && goal_y_.data == y_obs)
        {
            ROS_ERROR("Goal point is in collision with the obstacle!");
            return false;
        }
    }
    // no problem found
    return true;
}

nav_msgs::Path planner::MotionPlanner::convert_path(ob::ProblemDefinition* pdef)
{
    nav_msgs::Path plannedPath;
    plannedPath.header.frame_id = "/map";
    
    // get the obtained path
    ob::PathPtr path = pdef->getSolutionPath();
    
    // print the path to screen
    path->print(std::cout);
    
    // convert to geometric path
    const auto *path_ = path.get()->as<og::PathGeometric>();
    
    // iterate over each position
    for(unsigned int i=0; i<path_->getStateCount(); ++i){
        // get state
        const ob::State* state = path_->getState(i);
        // get x coord of the robot
        const auto *coordX = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        // get y coord of the robot
        const auto *coordY = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
        
        // fill in the ROS PoseStamped structure...
        geometry_msgs::PoseStamped poseMsg;
        
        poseMsg.pose.position.x = coordX->values[0];
        poseMsg.pose.position.y = coordY->values[0];
        poseMsg.pose.position.z = 0.01;
        poseMsg.pose.orientation.w = 1.0;
        poseMsg.pose.orientation.x = 0.0;
        poseMsg.pose.orientation.y = 0.0;
        poseMsg.pose.orientation.z = 0.0;
        poseMsg.header.frame_id = "/map";
        poseMsg.header.stamp = ros::Time::now();
        // ... and add the pose to the path
        plannedPath.poses.push_back(poseMsg);
    }
    return plannedPath;
}