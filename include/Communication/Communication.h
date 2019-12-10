//
// Created by bob on 18-10-19.
//

#ifndef SRC_COMMUNICATION_H
#define SRC_COMMUNICATION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <nav_msgs/Odometry.h>
#include <ropod_ros_msgs/RoutePlannerAction.h>
#include <Definitions/Pose2D.h>
#include <Definitions/Ellipse.h>
#include <ed_gui_server/objPosVel.h>
#include <ed_gui_server/objsPosVel.h>
#include <Obstacles/Obstacles.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <list>

class Communication {
public:
    ros::Publisher measuredOdom_pub;
    ros::Publisher vel_pub;
    ros::Subscriber obstacles_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber amcl_pose_sub;
    ros::Subscriber ropod_debug_plan_sub;
    ros::Subscriber scan_sub;
    ros::Subscriber vel_sub;

    string baseFrame, odomFrame, globalFrame;
    tf::TransformListener tf_listener;

    bool planUpdated = false, positionAmclUpdated = false, odometryUpdated = false, obstaclesUpdated = false;
    bool initializedPositionAmcl = false, initializedOdometry = false, initializedScan = false, initialized = false;

    ropod_ros_msgs::RoutePlannerResult route;
    Pose2D measuredPose, measuredVelocity, measuredPoseAmcl, measuredCmd;
    Ellipse poseUncertainty;
    list<Pose2D> measuredVelocityList;
    int velocityAverageSamples = 2;
    vector<Vector2D> laserPoints;
    Obstacles *obstacles;

    vector<Vector2D> footprint_param;
    Pose2D footprintMiddlePose_param;
    double maxSpeed_param = 0;
    double maxAcceleration_param = 0;
    double wheelDistanceMiddle_param = 0;
    double tubeWallOffset_param = 0;
    double tubeExtraSpace_param = 0;
    int nTries_param = 0;
    double predictionTime_param = 0;
    double minPredictionDistance_param = 0;
    double predictionBiasFactor_param = 0;

    Communication() = default;
    Communication(ros::NodeHandle nroshndl);

    void checkInitialized();
    bool newAmclPosition();
    bool newOdometry();
    bool newPlan();
    bool newObstacles();
    void getOdomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
    void getAmclCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg);
    void getDebugRoutePlanCallback(const ropod_ros_msgs::RoutePlannerResultConstPtr &routeData);
    void getObstaclesCallback(const ed_gui_server::objsPosVelConstPtr &obstacles_msg);
    void getLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void setVel(geometry_msgs::Twist cmd_vel_msg);
    void getVelCallback(const geometry_msgs::TwistConstPtr &cmd_vel_msg);

};


#endif //SRC_COMMUNICATION_H
