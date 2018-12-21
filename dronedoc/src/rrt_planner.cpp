#include <dronedoc/rrt_planner.hpp>

RRTPlanner::RRTPlanner() :
    initialized_(false),
    space_(new ob::SE2StateSpace()),
    bounds_(2),
    cbounds_(3),
    ss_ptr_(nullptr)
{
    auto pathcb = boost::bind(&RRTPlanner::globalPathCallback, this, _1);
    pathSub_ = nh_.subscribe<nav_msgs::Path>("/move_base/NavfnROS/plan", 10, pathcb);
    auto mapcb = boost::bind(&RRTPlanner::occupancyGridCallback, this, _1);
    mapSub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 10, mapcb);
}

RRTPlanner::~RRTPlanner(){}

void RRTPlanner::globalPathCallback(const nav_msgs::Path::ConstPtr &msg){
    ROS_INFO("New plan arrived");

    computeVelocityCommands(msg->poses[0]);
}

void RRTPlanner::occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    ROS_INFO("New map obtained");
    map_ = *msg;

    bounds_.setLow(0, -(map_.info.width/2.0));
    bounds_.setHigh(0, map_.info.width/2.0);
    bounds_.setLow(1, -(map_.info.height/2.0));
    bounds_.setHigh(1, map_.info.height/2.0);
    space_->as<ob::SE2StateSpace>()->setBounds(bounds_);
}

bool RRTPlanner::isStateValid(const ob::State *state){
    const ob::SE2StateSpace::StateType *state_2d = state->as<ob::SE2StateSpace::StateType>();
    const int &x(std::round(state_2d->getX()));
    const int &y(std::round(state_2d->getY()));

    if(map_.data[y*map_.info.width+x] > 49) {
        return false;
    }

    return true;
}

void RRTPlanner::propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result){
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

void RRTPlanner::initialize(){
    ROS_INFO("Initialize planner");

    bounds_.setLow(0, -(map_.info.width/2.0));
    bounds_.setHigh(0, map_.info.width/2.0);
    bounds_.setLow(1, -(map_.info.height/2.0));
    bounds_.setHigh(1, map_.info.height/2.0);
    space_->as<ob::SE2StateSpace>()->setBounds(bounds_);

    cbounds_.setLow(0, 0.0);
    cbounds_.setHigh(0, 0.5);
    cbounds_.setLow(1, 0.0);
    cbounds_.setHigh(1, 0.5);
    cbounds_.setLow(2, -2.0);
    cbounds_.setHigh(2, 2.0);
    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space_, 3));
    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds_);

    ss_ptr_.reset(new oc::SimpleSetup(cspace));
    auto stateValidityChecker = boost::bind(&RRTPlanner::isStateValid,
                                            this, _1);
    ss_ptr_->setStateValidityChecker(stateValidityChecker);
    auto propagate = boost::bind(&RRTPlanner::propagate,
                                 this, _1, _2, _3, _4);
    ss_ptr_->setStatePropagator(propagate);


    ss_ptr_->getSpaceInformation()->setMinMaxControlDuration(1, 50);
    ss_ptr_->getSpaceInformation()->setPropagationStepSize(0.1);

    ob::PlannerPtr planner(new oc::RRT(ss_ptr_->getSpaceInformation()));
    ss_ptr_->setPlanner(planner);
}

void RRTPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped &pose){
    ROS_INFO("Start planning");

    tf::StampedTransform transform;

    try{
        tf_listener_.lookupTransform("/base_link",
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

    ob::ScopedState<ob::SE2StateSpace> start(space_);
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

    ob::ScopedState<ob::SE2StateSpace> goal(space_);
    goal->setXY(goal_x, goal_y);
    goal->setYaw(goalYaw);
    ROS_INFO("goal: ");
    goal.print(std::cout);

    ss_ptr_->setStartAndGoalStates(start, goal, 0.1);

    ob::PlannerStatus solved = ss_ptr_->solve(3.0);

    if(solved){
        ROS_INFO("Solution Found:");

        ss_ptr_->getSolutionPath().print(std::cout);
    }else{
        ROS_INFO("No solution found");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_planner");
    ros::NodeHandle nh;
    ros::Rate rate(20);

    ROS_INFO("Create planner instance");
    RRTPlanner planner;
    ROS_INFO("Initialize");
    planner.initialize();

    ros::spin();

    return 0;
}