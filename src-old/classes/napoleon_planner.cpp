#include "napoleon_planner.h"

void NapoleonPlanner::executeCB(const ropod_ros_msgs::GoToGoalConstPtr &goal)
{
    std::vector<ropod_ros_msgs::Area> area_list = goal->action.areas;
    ropod_ros_msgs::RoutePlannerGoal route_planner_goal;
    route_planner_goal.areas = area_list;
    ac_.sendGoal(route_planner_goal, boost::bind(&NapoleonPlanner::plannerResultCB, this, _1, _2));

    //wait for the action to return
    bool finished_before_timeout = ac_.waitForResult(ros::Duration(60.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac_.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
        status_ = true;
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
        status_ = false;
    }
}

void NapoleonPlanner::plannerResultCB(const actionlib::SimpleClientGoalState& state, const ropod_ros_msgs::RoutePlannerResultConstPtr& result)
{
    route_planner_result_ = *result;
}

bool NapoleonPlanner::getStatus()
{
    return status_;
}
