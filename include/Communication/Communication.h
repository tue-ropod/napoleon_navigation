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
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <queue>
#include <sensor_msgs/LaserScan.h>

class Communication {
public:

    ros::Subscriber obstacles_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber amcl_pose_sub;
    ros::Subscriber ropod_debug_plan_sub;
    ros::Publisher vel_pub;
    ros::Subscriber scan_sub;

    sensor_msgs::LaserScan::ConstPtr scan;
    std::queue<sensor_msgs::LaserScan::ConstPtr> scan_buffer;
    tf::TransformListener *tf_listener_;
    ropod_ros_msgs::RoutePlannerResult route;
    Pose2D measuredPose, measuredVelocity;
    vector<Vector2D> laserPoints;
    Obstacles obstacles;
    bool planUpdated = false;
    bool positionUpdated = false;
    bool odometryUpdated = false;
    bool initializedPosition = false, initializedOdometry = false, initialized = false;

    bool updatePosition = true;
    bool communicate = true;

    Communication(ros::NodeHandle nroshndl);

    void checkInitialized();
    bool newPosition();
    bool newOdometry();
    bool newPlan();
    void getOdomVelCallback(const nav_msgs::OdometryConstPtr &odom_msg);
    void getAmclPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg);
    void getDebugRoutePlanCallback(const ropod_ros_msgs::RoutePlannerResultConstPtr &routeData);
    void getObstaclesCallback(const ed_gui_server::objsPosVelConstPtr &obstacles_msg);
    void getLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void setVel(geometry_msgs::Twist cmd_vel_msg);

};


#endif //SRC_COMMUNICATION_H
