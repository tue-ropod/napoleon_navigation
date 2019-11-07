//
// Created by bob on 18-10-19.
//

#include "Communication.h"

Communication::Communication(ros::NodeHandle nroshndl) {
    if(!updatePosition){initializedPosition = true;}
    if(communicate) {
        vel_pub = nroshndl.advertise<geometry_msgs::Twist>("/remap/cmd_vel", 1);
        amcl_pose_sub = nroshndl.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1, &Communication::getAmclPoseCallback, this);
        odom_sub = nroshndl.subscribe<nav_msgs::Odometry>("/remap/odom", 1, &Communication::getOdomVelCallback, this);
        obstacles_sub = nroshndl.subscribe<ed_gui_server::objsPosVel>("/ed/gui/objectPosVel", 1, &Communication::getObstaclesCallback, this);
        //ros::Subscriber goal_cmd_sub = nroshndl.subscribe<geometry_msgs::PoseStamped>("/route_navigation/simple_goal", 10, simpleGoalCallback);
        ropod_debug_plan_sub = nroshndl.subscribe<ropod_ros_msgs::RoutePlannerResult>("/ropod/debug_route_plan", 1, &Communication::getDebugRoutePlanCallback, this);
        scan_sub = nroshndl.subscribe<sensor_msgs::LaserScan>("/remap/scan", 1, &Communication::getLaserScanCallback, this);
        tf_listener_ = new tf::TransformListener;
    }
}

void Communication::getOdomVelCallback(const nav_msgs::OdometryConstPtr &odom_msg){
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

void Communication::getAmclPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg){
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

void Communication::getObstaclesCallback(const ed_gui_server::objsPosVelConstPtr &obstacles_msg) {
    obstacles.obstacles.clear();
    for(auto & obstacle : obstacles_msg->objects){
        if(obstacle.rectangle.probability > obstacle.circle.probability) {
            double x = obstacle.rectangle.pose.position.x;
            double y = obstacle.rectangle.pose.position.y;
            double angle = obstacle.rectangle.yaw;

            Pose2D pose = Pose2D(x, y, angle);
            Vector2D p1 = Vector2D(-obstacle.rectangle.width / 2, -obstacle.rectangle.depth / 2);
            Vector2D p2 = Vector2D(obstacle.rectangle.width / 2, -obstacle.rectangle.depth / 2);
            Vector2D p3 = Vector2D(obstacle.rectangle.width / 2, obstacle.rectangle.depth / 2);
            Vector2D p4 = Vector2D(-obstacle.rectangle.width / 2, obstacle.rectangle.depth / 2);
            Polygon shape = Polygon({p1, p2, p3, p4}, Closed);

            Obstacle obs = Obstacle(shape, pose, Dynamic);
            obs.movement = Pose2D(obstacle.rectangle.vel.x, obstacle.rectangle.vel.y, 0);
            obstacles.obstacles.emplace_back(obs);
        }else{
            double x = obstacle.circle.pose.position.x;
            double y = obstacle.circle.pose.position.y;
            double r = obstacle.circle.radius;

            Pose2D pose = Pose2D(x,y,0);
            Circle circle(Vector2D(0,0),r);
            Polygon shape = circle.toPoints(8);

            Obstacle obs = Obstacle(shape, pose, Dynamic);
            obs.movement = Pose2D(obstacle.rectangle.vel.x, obstacle.rectangle.vel.y, 0);
            obstacles.obstacles.emplace_back(obs);
        }
    }
}

void Communication::setVel(geometry_msgs::Twist cmd_vel_msg){
    if(communicate) {
        vel_pub.publish(cmd_vel_msg);
    }
}

void Communication::getDebugRoutePlanCallback(const ropod_ros_msgs::RoutePlannerResultConstPtr &routeData){
    route = *routeData;
    ROS_INFO("new debug plan received");
    planUpdated = true;
}

void Communication::getLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    scan_buffer.push(msg);
    bool newData = false;
    while(!scan_buffer.empty()){
        scan = scan_buffer.front();
        // - - - - - - - - - - - - - - - - - -
        // Determine absolute laser pose based on TF
        try{
            tf::StampedTransform t_sensor_pose;
            tf_listener_->lookupTransform("map", scan->header.frame_id, scan->header.stamp, t_sensor_pose);
            scan_buffer.pop();

            tf::Quaternion q = t_sensor_pose.getRotation(); //( t_sensor_pose.getRotation().x, t_sensor_pose.getRotation().y, t_sensor_pose.getRotation().z, t_sensor_pose.getRotation().w );
            tf::Matrix3x3 matrix ( q );
            double rollSensor, pitchSensor, yawSensor;
            matrix.getRPY ( rollSensor, pitchSensor, yawSensor );

            double scan_size =  scan->ranges.size();
            laserPoints.clear();
            for(unsigned int iScan = 0; iScan < scan_size; iScan ++){
                double angle = yawSensor + scan->angle_min + scan->angle_increment*iScan;
                double x = t_sensor_pose.getOrigin().getX() + scan->ranges[iScan]*cos( angle );
                double y = t_sensor_pose.getOrigin().getY() + scan->ranges[iScan]*sin( angle );
                laserPoints.emplace_back(Vector2D(x,y));
            }
            newData = true;
        }
        catch(tf::ExtrapolationException& ex){
            //ROS_WARN_STREAM_DELAYED_THROTTLE(10, "ED Laserplugin tracking: " << ex.what());
            try{
                // Now we have to check if the error was an interpolation or extrapolation error
                // (i.e., the scan is too old or too new, respectively)
                tf::StampedTransform latest_transform;
                tf_listener_->lookupTransform("map", scan->header.frame_id, ros::Time(0), latest_transform);

                if (scan_buffer.front()->header.stamp > latest_transform.stamp_){
                    // Scan is too new
                    break;
                }
                else{
                    // Otherwise it has to be too old (pop it because we cannot use it anymore)
                    scan_buffer.pop();
                }
            }
            catch(tf::TransformException& exc){
                scan_buffer.pop();
            }
        }
        catch(tf::TransformException& exc){
            //ROS_ERROR_STREAM_DELAYED_THROTTLE(10, "ED Laserplugin tracking: " << exc.what());
            scan_buffer.pop();
        }
    }
    if(!laserPoints.empty() && newData){
        obstacles.obstacles.clear();
        int counter = 0;
        Vector2D p1, p2;
        for(int i = 0; i < laserPoints.size()-1; i++){
            if(counter == 0){p1 = laserPoints[i];}
            if((laserPoints[i]-laserPoints[i+1]).length() < 0.1 && counter <= 20){
                counter++;
                p2 = laserPoints[i+1];
            }else{
                if(counter >= 5){
                    obstacles.obstacles.emplace_back(Obstacle(Polygon({p1, p2}, Open), Pose2D((p1+p2)/2, 0), Static));
                }
                counter = 0;
            }
        }
    }
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
