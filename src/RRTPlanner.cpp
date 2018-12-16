#include <ompl/control/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>

#include <cmath>
#include <vector>

#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

void plan(const geometry_msgs::PoseStamped&);

void globalPathCallback(const nav_msgs::Path::ConstPtr &msg){
    ROS_INFO("New plan arrived");

    std::vector<geometry_msgs::PoseStamped> poses = msg->poses;

    ROS_INFO("Path size: %d", poses.size());
    for (const auto &pose: poses){
        plan(pose);
    }
}

nav_msgs::OccupancyGrid map;
void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    ROS_INFO("New map obtained");
    map = *msg;
}

bool isStateValid(const ob::State *state){
    const ob::SE2StateSpace::StateType *state_2d = state->as<ob::SE2StateSpace::StateType>();
    const int &x(std::round(state_2d->getX()));
    const int &y(std::round(state_2d->getY()));

    if(map.data[y*map.info.width+x] > 49) {
        return false;
    }

    return true;
}

template<typename T>
inline T FMod(const T &x, const T &y){
    if(y==0) return x;
    return x-y*std::floor(x/y);
}

double RadToNPiPPi(const double &x){
    return FMod(x+M_PI, M_PI*2.0)-M_PI;
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result){
    const ob::SE2StateSpace::StateType *start_2d = start->as<ob::SE2StateSpace::StateType>();
    const double &x(start_2d->getX()), &y(start_2d->getY()), &rot(start_2d->getYaw());
    const oc::RealVectorControlSpace::ControlType *control_3 = control->as<oc::RealVectorControlSpace::ControlType>();
    const double &c0((*control_3)[0]), &c1((*control_3)[1]), &c2((*control_3)[2]);
    ob::SE2StateSpace::StateType *result_2d = result->as<ob::SE2StateSpace::StateType>();

    double nx = x + c0*duration;
    double ny = y + c1*duration;
    if(nx<-1.0) nx = -1.0; else if(nx>1.0) nx = 1.0;
    if(ny<-1.0) ny = -1.0; else if(ny>1.0) ny = 1.0;
    result_2d->setXY(nx, ny);
    result_2d->setYaw(RadToNPiPPi(rot+c2*duration));
}

void plan(const geometry_msgs::PoseStamped &pose){
    ROS_INFO("Start planning");
    ob::StateSpacePtr space(new ob::SE2StateSpace());
    ob::RealVectorBounds bounds(2);

    bounds.setLow(0, -(map.info.width/2.0));
    bounds.setHigh(0, map.info.width/2.0);
    bounds.setLow(1, -(map.info.height/2.0));
    bounds.setHigh(1, map.info.height/2.0);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);
    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 3));
    ob::RealVectorBounds cbounds(3);
    cbounds.setLow(0, 0.0);
    cbounds.setHigh(0, 0.5);
    cbounds.setLow(1, 0.0);
    cbounds.setHigh(1, 0.5);
    cbounds.setLow(2, -2.0);
    cbounds.setHigh(2, 2.0);
    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    oc::SimpleSetup ss(cspace);
    ss.setStateValidityChecker(boost::bind(&isStateValid, _1));
    ss.setStatePropagator(boost::bind(&propagate, _1, _2, _3, _4));

    tf::TransformListener tf_listener;
    tf::StampedTransform transform;

    try{
        tf_listener.lookupTransform("/base_link",
                                    "/map",
                                    ros::Time(0),
                                    transform);
    }catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    double startX = transform.getOrigin().getX();
    double startY = transform.getOrigin().getY();
    tf::Matrix3x3 startRot(transform.getRotation());
    double startR, startP, startYaw;
    startRot.getRPY(startR, startP, startYaw);

    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setXY(startX, startY);
    start->setYaw(startYaw);
    ROS_INFO("start: ");
    start.print(std::cout);

    double goal_x = pose.pose.position.x;
    double goal_y = pose.pose.position.y;
    tf::Quaternion goalQuat(pose.pose.orientation.x,
                            pose.pose.orientation.y,
                            pose.pose.orientation.z,
                            pose.pose.orientation.w);
    tf::Matrix3x3 goalRot(goalQuat);
    double goalR, goalP, goalYaw;
    goalRot.getRPY(goalR, goalP, goalYaw);

    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setXY(goal_x, goal_y);
    goal->setYaw(goalYaw);
    ROS_INFO("goal: ");
    goal.print(std::cout);

    ss.setStartAndGoalStates(start, goal, 0.1);

    ss.getSpaceInformation()->setMinMaxControlDuration(1, 50);
    ss.getSpaceInformation()->setPropagationStepSize(0.1);

    ob::PlannerStatus solved = ss.solve(5.0);

    if(solved){
        ROS_INFO("Solution Found:");

        ss.getSolutionPath().print(std::cout);

        std::ofstream ofs("path.dat");
        ss.getSolutionPath().printAsMatrix(ofs);
    }else{
        ROS_INFO("No solution found");
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "RRTPlanner");
    ros::NodeHandle nh;
    ros::Rate rate(20);

    ros::Subscriber pathSub = nh.subscribe<nav_msgs::Path>("/move_base/NavfnROS/plan", 10, globalPathCallback);
    ros::Subscriber mapSub = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 10, occupancyGridCallback);

    ros::spin();

    return 0;
}