#ifndef INCLUDED_rrt_planner_hpp_
#define INCLUDED_rrt_planner_hpp_

#include <cmath>

#include <ompl/control/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class RRTPlanner{

public:
    /**
     * @brief Default constructor
     */
    RRTPlanner();

    /**
     * @brief Destructor
     */
    ~RRTPlanner();

    void initialize();

    void computeVelocityCommands(const geometry_msgs::PoseStamped &pose);

    /**
     * @brief Return whether planner is initialized
     * @return True if planner is initialized
     */
    bool isInitialized()
    {
        return initialized_;
    };

private:
    bool isStateValid(const ob::State *state);

    void propagate(const ob::State *start,
                   const oc::Control *control,
                   const double duration,
                   ob::State *result);

    void globalPathCallback(const nav_msgs::Path::ConstPtr &msg);

    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    template<typename T>
    T FMod(const T &x, const T &y){
        if(y==0) return x;
        return x-y*std::floor(x/y);
    }

    double RadToNPiPPi(const double &x){
        return FMod(x+M_PI, M_PI*2.0)-M_PI;
    }

    //! True if planner is initialized
    bool initialized_;

    nav_msgs::OccupancyGrid map_;

    ob::StateSpacePtr space_;
    std::unique_ptr<oc::SimpleSetup> ss_ptr_;

    ob::RealVectorBounds bounds_;
    ob::RealVectorBounds cbounds_;
    ros::NodeHandle nh_;

    tf::TransformListener tf_listener_;

    ros::Subscriber pathSub_;
    ros::Subscriber mapSub_;
};

#endif