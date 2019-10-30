//
// Created by bob on 18-10-19.
//

#include "Communication.h"

Communication::Communication(ros::NodeHandle nroshndl, bool updatePosition_) {
    updatePosition = updatePosition_;
    if(!updatePosition){initializedPosition = true;}
    vel_pub = nroshndl.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    amcl_pose_sub = nroshndl.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &Communication::getAmclPoseCallback, this);
    odom_sub = nroshndl.subscribe<nav_msgs::Odometry>("/ropod/odom", 100, &Communication::getOdomVelCallback, this);
    obstacles_sub = nroshndl.subscribe<ed_gui_server::objsPosVel>("/ed/gui/objectPosVel", 10, &Communication::getObstaclesCallback, this);
    //ros::Subscriber goal_cmd_sub = nroshndl.subscribe<geometry_msgs::PoseStamped>("/route_navigation/simple_goal", 10, simpleGoalCallback);
    ropod_debug_plan_sub = nroshndl.subscribe< ropod_ros_msgs::RoutePlannerResult >("/ropod/debug_route_plan", 1, &Communication::getDebugRoutePlanCallback, this);
}

void Communication::getOdomVelCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    //ROS_INFO("Odometry received");
    double odom_xdot_ropod_global = abs(odom_msg->twist.twist.linear.x) < 1e-5 ? 0.0 : odom_msg->twist.twist.linear.x;
    double odom_ydot_ropod_global = abs(odom_msg->twist.twist.linear.y) < 1e-5 ? 0.0 : odom_msg->twist.twist.linear.y;
    double odom_thetadot_global = abs(odom_msg->twist.twist.angular.z) < 1e-5 ? 0.0 : odom_msg->twist.twist.angular.z;
    //double odom_phi_local = atan2(odom_ydot_ropod_global, odom_xdot_ropod_global);
    //double odom_vropod_global = sqrt(odom_xdot_ropod_global*odom_xdot_ropod_global+odom_ydot_ropod_global*odom_ydot_ropod_global);

    measuredVelocity = Pose2D(odom_xdot_ropod_global, odom_ydot_ropod_global, odom_thetadot_global);

    if(!initializedOdometry){
        cout << "Initial Odom: " << measuredVelocity.x << " " << measuredVelocity.y << " " << measuredVelocity.a << endl;
        initializedOdometry = true;
        checkInitialized();
    }

    odometryUpdated = true;
}

void Communication::getAmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg){
    if(updatePosition) {
        //ROS_INFO("Amcl pose received");
        //ROS_INFO("X: %f, Y: %f", pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
        double this_amcl_x = pose_msg->pose.pose.position.x;
        double this_amcl_y = pose_msg->pose.pose.position.y;
        double quaternion_x = pose_msg->pose.pose.orientation.x;
        double quaternion_y = pose_msg->pose.pose.orientation.y;
        double quaternion_z = pose_msg->pose.pose.orientation.z;
        double quaternion_w = pose_msg->pose.pose.orientation.w;

        // yaw (z-axis rotation)
        double siny_cosp = +2.0 * (quaternion_w * quaternion_z + quaternion_x * quaternion_y);
        double cosy_cosp = +1.0 - 2.0 * (quaternion_y * quaternion_y + quaternion_z * quaternion_z);
        double this_amcl_theta = atan2(siny_cosp, cosy_cosp);

        measuredPose = Pose2D(this_amcl_x, this_amcl_y, this_amcl_theta);
        //cout << "Pose: " << pose.x << " " << pose.y << " " << pose.a << endl;

        if (!initializedPosition) {
            cout << "Initial pose: " << measuredPose.x << " " << measuredPose.y << " " << measuredPose.a << endl;
            initializedPosition = true;
            checkInitialized();
        }

        positionUpdated = true;
    }
}

void Communication::getObstaclesCallback(const ed_gui_server::objsPosVel::ConstPtr& obstacles_msg) {
    obstacles.obstacles.clear();
    cout << "Obstacles: " << obstacles_msg->objects.size() << endl;
    for(auto & obstacle : obstacles_msg->objects){
        double x = obstacle.rectangle.pose.position.x;
        double y = obstacle.rectangle.pose.position.y;
        double angle = obstacle.rectangle.yaw;
        Pose2D pose = Pose2D(x, y, angle);

        Vector2D p1 = Vector2D(-obstacle.rectangle.width/2, -obstacle.rectangle.depth/2);
        Vector2D p2 = Vector2D(obstacle.rectangle.width/2, -obstacle.rectangle.depth/2);
        Vector2D p3 = Vector2D(obstacle.rectangle.width/2, obstacle.rectangle.depth/2);
        Vector2D p4 = Vector2D(-obstacle.rectangle.width/2, obstacle.rectangle.depth/2);
        Polygon footprint = Polygon({p1, p2, p3, p4}, Closed);

        Obstacle obs = Obstacle(footprint, pose, Dynamic);
        obs.movement = Pose2D(obstacle.rectangle.vel.x, obstacle.rectangle.vel.y, 0);

        obstacles.obstacles.emplace_back(obs);
    }
}

void Communication::setVel(geometry_msgs::Twist cmd_vel_msg){
    vel_pub.publish(cmd_vel_msg);
}

void Communication::getDebugRoutePlanCallback(const ropod_ros_msgs::RoutePlannerResultConstPtr& routeData){
    route = *routeData;
    ROS_INFO("new debug plan received");
    planUpdated = true;
}

void Communication::checkInitialized(){
    initialized = initializedOdometry && initializedPosition;
}

bool Communication::newPosition(){
    bool temp = positionUpdated;
    positionUpdated = false;
    return temp;
}

bool Communication::newOdometry(){
    bool temp = odometryUpdated;
    odometryUpdated = false;
    return temp;
}

bool Communication::newPlan(){
    bool temp = planUpdated;
    planUpdated = false;
    return temp;
}