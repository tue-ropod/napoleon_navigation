//
// Created by bob on 25-09-19.
//

#include "Model.h"

Model::Model(Pose2D pose_, Polygon footprint_, double maxSpeed_, double maxAcceleration_, double wheelDistanceToMiddle_){
    pose = pose_;
    footprint = std::move(footprint_);
    dilatedFootprint = footprint;
    footprint.transformto(pose);
    dilatedFootprint = footprint;
    scanradius = Circle(Vector2D(pose.x, pose.y), 30);
    maxSpeed = maxSpeed_;
    maxAcceleration = maxAcceleration_;
    maxRotationalSpeed = maxSpeed / wheelDistanceToMiddle_;
    maxRotationalAcceleration = maxAcceleration / wheelDistanceToMiddle_;
    currentTubeIndex = 0;
    speedScale = 1;
}

void Model::subscribe(ros::NodeHandle nroshndl){
    vel_pub = nroshndl.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    amcl_pose_sub = nroshndl.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &Model::getAmclPoseCallback, this);
    ropod_odom_sub = nroshndl.subscribe<nav_msgs::Odometry>("/ropod/odom", 100, &Model::getOdomVelCallback, this);
}

bool Model::collision(Obstacle& o){
    //sequence only check sides collisions of a polygon if the middle is not inside.
    return (footprint.polygonContainsPoint(o.footprint.middle) || footprint.polygonPolygonCollision(o.footprint));
}

void Model::scaleFootprint(double x, double y) {
    footprint.transformto(Pose2D(0,0,0));
    footprint.scale(1/footprintscalex, 1/footprintscaley);
    footprint.scale(x, y);
    footprint.transformto(pose);
    footprintscalex = x;
    footprintscaley = y;
}

void Model::dilateFootprint(double offset){
    dilatedFootprint.dilate(offset);
}

void Model::changeSpeedScale(double x) {
    speedScale += x;
    speedScale = speedScale > 1 ? 1 : speedScale;
    speedScale = speedScale < 0 ? 0 : speedScale;
}

void Model::copySettings(Model &modelCopy) {
    speedScale = modelCopy.speedScale;
}

void Model::copyState(Model &modelCopy) {
    velocity = modelCopy.velocity;
    inputVelocity = modelCopy.inputVelocity;
    pose = modelCopy.pose;
    footprint.transformto(pose);
    dilatedFootprint.transformto(pose);
}

void Model::update(double dt) {
    Pose2D acc = (inputVelocity - velocity)/dt;
    if(acc.length() > maxAcceleration){
        Vector2D scaledAcc = acc.unit()*maxAcceleration;
        acc.x = scaledAcc.x;
        acc.y = scaledAcc.y;
    }

    if(abs(acc.a) > maxRotationalAcceleration){
        acc.a = (acc.a/abs(acc.a))*maxRotationalAcceleration;
    }

    velocity = velocity + acc * dt;

    if(velocity.length() > maxSpeed){
        Vector2D scaledVel = velocity.unit()*maxSpeed;
        velocity.x = scaledVel.x;
        velocity.y = scaledVel.y;
    }
    if(abs(velocity.a) > maxRotationalSpeed){
        velocity.a = (velocity.a/abs(velocity.a))*maxRotationalSpeed;
    }

    setSpeed(velocity);
}

void Model::setSpeed(Pose2D vel){
    vel.transformThis(0,0,-M_PI_2);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vel.x;
    cmd_vel.linear.y = vel.y;
    cmd_vel.angular.z = vel.a;
    vel_pub.publish(cmd_vel);
}

FollowStatus Model::predict(int nScalings, double predictionTime, double minPredictionDistance, double dt, Model &origionalModel, Tubes &tubes, Visualization &canvas) {
    FollowStatus status = Status_Error;

    changeSpeedScale(1/double(nScalings));

    for(int s = 0; s < nScalings; s++) {
        copyState(origionalModel);
        Vector2D prevPos = pose.toVector();
        double distance = 0;
        for (int p = 0; p < ceil(predictionTime / dt); p++) {
            status = follow(tubes, canvas, false);
            updatePrediction(dt);
            distance += prevPos.distance(pose.toVector());
            prevPos = pose.toVector();
            //show(canvas, Color(255,255,255), Thin);
            if (status != Status_Ok) { break; }
        }
        //show(canvas, Color(255, 255, 255), Thin);
        if (distance < minPredictionDistance) {
            status = Status_ToClose;
        }
        if (status != Status_Ok && status != Status_Done) {
            changeSpeedScale(-1/double(nScalings));
            if (speedScale == 0) { break; }
        } else { break; }
    }

    show(canvas, Color(255, 255, 255), Thin);

    return status;
}

////////////////////////////////////Virtual functions///////////////////////////////////////////////////////////////////

void Model::show(Visualization& canvas, Color c, int drawstyle) {
    canvas.polygon(footprint.vertices, c, drawstyle);
    canvas.polygon(dilatedFootprint.vertices, Color(255,0,0), Thin);
}

FollowStatus Model::follow(Tubes &tubes, Visualization &canvas, bool debug) {
    return Status_Error;
}

void Model::updatePrediction(double dt){}

void Model::input(Pose2D velocity_, Frame frame){}

Pose2D Model::translateInput(Vector2D position, Pose2D velocity_){
    return {};
}

///////////////////////////////////Ros communication////////////////////////////////////////////////////////////////////

void Model::getOdomVelCallback(const nav_msgs::Odometry::ConstPtr& odom_vel){
    //ROS_INFO("Odometry received");
    double odom_xdot_ropod_global = odom_vel->twist.twist.linear.x;
    double odom_ydot_ropod_global = odom_vel->twist.twist.linear.y;
    double odom_thetadot_global = odom_vel->twist.twist.angular.z;
    //double odom_phi_local = atan2(odom_ydot_ropod_global, odom_xdot_ropod_global);
    //double odom_vropod_global = sqrt(odom_xdot_ropod_global*odom_xdot_ropod_global+odom_ydot_ropod_global*odom_ydot_ropod_global);

    velocity = Pose2D(odom_xdot_ropod_global, odom_ydot_ropod_global, odom_thetadot_global);
    //cout << "Velocity: " << velocity.x << " " << velocity.y << " " << velocity.a << endl;
}

void Model::getAmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg){
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

    pose = Pose2D(this_amcl_x, this_amcl_y, this_amcl_theta);
    //cout << "Pose: " << pose.x << " " << pose.y << " " << pose.a << endl;
    footprint.transformto(pose);
    dilatedFootprint.transformto(pose);
}

