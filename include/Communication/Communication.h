//
// Created by bob on 18-10-19.
//

#ifndef SRC_COMMUNICATION_H
#define SRC_COMMUNICATION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ropod_ros_msgs/RoutePlannerAction.h>
#include <Definitions/Pose2D.h>
#include <ed_gui_server/objPosVel.h>
#include <ed_gui_server/objsPosVel.h>
#include <Obstacles/Obstacles.h>

class Communication {
public:

    ros::Subscriber obstacles_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber amcl_pose_sub;
    ros::Subscriber ropod_debug_plan_sub;
    ros::Publisher vel_pub;
    ropod_ros_msgs::RoutePlannerResult route;
    Pose2D measuredPose, measuredVelocity;
    Obstacles obstacles;
    bool planUpdated = false;
    bool positionUpdated = false;
    bool odometryUpdated = false;
    bool initializedPosition = false, initializedOdometry = false, initialized = false;
    bool updatePosition;

    Communication(ros::NodeHandle nroshndl, bool updatePosition_ = true);

    void checkInitialized();
    bool newPosition();
    bool newOdometry();
    bool newPlan();
    void getOdomVelCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    void getAmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg);
    void getDebugRoutePlanCallback(const ropod_ros_msgs::RoutePlannerResultConstPtr& routeData);
    void getObstaclesCallback(const ed_gui_server::objsPosVel::ConstPtr& obstacles_msg);
    void setVel(geometry_msgs::Twist cmd_vel_msg);

};


#endif //SRC_COMMUNICATION_H
