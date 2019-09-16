#ifndef NAP_TUB_H
#define NAP_TUB_H

using namespace std;

#include <vector>
#include "napoleon_geometry.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <ropod_ros_msgs/GoToAction.h>
#include <ropod_ros_msgs/RoutePlannerAction.h>

class NapoleonPlanner
{
public:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ropod_ros_msgs::GoToAction> as_;
    actionlib::SimpleActionClient<ropod_ros_msgs::RoutePlannerAction> ac_;
    std::string action_name_;
    ropod_ros_msgs::GoToFeedback feedback_;
    ropod_ros_msgs::GoToResult result_;
    ropod_ros_msgs::RoutePlannerResult route_planner_result_;
    bool status_;

public:
    NapoleonPlanner(std::string name) :
        as_(nh_, name, boost::bind(&NapoleonPlanner::executeCB, this, _1), false),
        action_name_(name), ac_("/route_planner", true), status_(false)
    {
        ROS_INFO("Waiting for route planner action server to start");
        ac_.waitForServer();
        ROS_INFO("Connected to route planner action server");
        // waiting for route planner action server to start
        as_.start();
        ROS_INFO("Waiting for GOTO action");
    }

    ~NapoleonPlanner(void){}

    ropod_ros_msgs::RoutePlannerResult getPlannerResult()
    {
        return route_planner_result_;
    }

    bool getStatus();
    void plannerResultCB(const actionlib::SimpleClientGoalState& state, const ropod_ros_msgs::RoutePlannerResultConstPtr& result);
    void executeCB(const ropod_ros_msgs::GoToGoalConstPtr &goal);
};

#endif
