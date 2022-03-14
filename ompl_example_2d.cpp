/*
 * controllerUR_node.cpp
 *
 *  Created on: Jun 3, 2017
 *      Author: Dominik Belter
 *   Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#include "../include/ompl_example_2d/ompl_example_2d.hpp"

// Boost
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>
#include <math.h>
#include <limits>
#include <thread>

using namespace std;
using namespace ros;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_example_2d {

/// occupancy map used for planning
nav_msgs::OccupancyGrid occupancyMap;

Planner2D::Planner2D(ros::NodeHandle& _nodeHandle)
    : nodeHandle(_nodeHandle)
{
    ROS_INFO("Controlling UR node started.");
    configure();
}

Planner2D::~Planner2D()
{
}

/// check if the current state is valid
bool isStateValid(const ob::State *state){
    // get x coord of the robot
    const auto *coordX =
            state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    // get y coord of the robot
    const auto *coordY =
            state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

    if(occupancyMap.info.width>0&&occupancyMap.info.height>0){
       int x_1 = (coordX->values[0] - occupancyMap.info.origin.position.x)/occupancyMap.info.resolution-5;
       int y_1 = (coordY->values[0] - occupancyMap.info.origin.position.y)/occupancyMap.info.resolution-5;
       int x_2 = (coordX->values[0] - occupancyMap.info.origin.position.x)/occupancyMap.info.resolution+5;
       int y_2 = (coordY->values[0] - occupancyMap.info.origin.position.y)/occupancyMap.info.resolution+5;

       for(int i = y_1;i<y_2;i++)
       {
           for(int j = x_1;j<x_2;j++)
           {
              float data = occupancyMap.data[j+i*occupancyMap.info.width];
              if (data>50)
                   {  //||data<0 dodatkowy warunek do zadania pierwszego (plener nie porusza się po komórkach nieznanych )
                     return false;
                   }
           }
       }
    }
    else
    {
        return false;
    }
    return true;
}

/// extract path
nav_msgs::Path Planner2D::extractPath(ob::ProblemDefinition* pdef){
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
        const auto *coordX =
                state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
        // get y coord of the robot
        const auto *coordY =
                state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
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

/*!
 * plan path
 */
nav_msgs::Path Planner2D::planPath(const nav_msgs::OccupancyGrid& globalMap){
    occupancyMap = globalMap;

    // search space information
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));
    // define state checking callback
    si->setStateValidityChecker(isStateValid);
    // set State Validity Checking Resolution (avoid going through the walls)
    si->setStateValidityCheckingResolution(0.001);

    // problem definition
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(*start.get(), *goal.get());

    // create planner
    auto planner(std::make_shared<og::RRTConnect>(si));
    // configure the planner
    planner->setRange(maxStepLength);// max step length
    planner->setProblemDefinition(pdef);
    planner->setup();

    // solve motion planning problem
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    nav_msgs::Path plannedPath;
    if (solved) {// if cussess
        // get the planned path
        plannedPath=extractPath(pdef.get());
    }
    return plannedPath;
}

/// configure planner
void Planner2D::configure(void){
    dim = 2;//2D problem
    maxStepLength = 0.1;// max step length

    // create bounds for the x axis
    coordXBound.reset(new ob::RealVectorBounds(dim-1));
    coordXBound->setLow(-1.0);
    coordXBound->setHigh(13.0);

    // create bounds for the y axis
    coordYBound.reset(new ob::RealVectorBounds(dim-1));
    coordYBound->setLow(-5.0);
    coordYBound->setHigh(5.0);

    // construct the state space we are planning in
    auto coordX(std::make_shared<ob::RealVectorStateSpace>(dim-1));
    auto coordY(std::make_shared<ob::RealVectorStateSpace>(dim-1));
    space = coordX +coordY;

    // create bounds for the x axis
    coordX->setBounds(*coordXBound.get());

    // create bounds for the y axis
    coordY->setBounds(*coordYBound.get());

    // define the start position
    start.reset(new ob::ScopedState<>(space));
    (*start.get())[0]=0.0;
    (*start.get())[1]=-2.5;
//    start.get()->random();

    // define the goal position
    goal.reset(new ob::ScopedState<>(space));
    (*goal.get())[0]=12.0;
    (*goal.get())[1]=-4.0;
//    goal.get()->random();
}

} /* namespace */
