#include "napoleon_model.h"

void NapoleonModel::getOdomVelCallback(const nav_msgs::Odometry::ConstPtr& odom_vel){
    odom_xdot_ropod_global = odom_vel->twist.twist.linear.x;
    odom_ydot_ropod_global = odom_vel->twist.twist.linear.y;
    odom_thetadot_global = odom_vel->twist.twist.angular.z;
    odom_phi_local = atan2(odom_ydot_ropod_global, odom_xdot_ropod_global);
    odom_vropod_global = sqrt(odom_xdot_ropod_global*odom_xdot_ropod_global+odom_ydot_ropod_global*odom_ydot_ropod_global);
    //ROS_INFO("xdot: %f, ydot: %f, vabs: %f", odom_xdot_ropod_global, odom_ydot_ropod_global, odom_vropod_global);
}

void NapoleonModel::getAmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg){
    //ROS_INFO("Amcl pose received");
    //ROS_INFO("X: %f, Y: %f", pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
    this_amcl_x = pose_msg->pose.pose.position.x;
    this_amcl_y = pose_msg->pose.pose.position.y;
    quaternion_x = pose_msg->pose.pose.orientation.x;
    quaternion_y = pose_msg->pose.pose.orientation.y;
    quaternion_z = pose_msg->pose.pose.orientation.z;
    quaternion_w = pose_msg->pose.pose.orientation.w;

    // yaw (z-axis rotation)
    siny_cosp = +2.0 * (quaternion_w * quaternion_z + quaternion_x * quaternion_y);
    cosy_cosp = +1.0 - 2.0 * (quaternion_y * quaternion_y + quaternion_z * quaternion_z);
    this_amcl_theta = atan2(siny_cosp, cosy_cosp);
}

void NapoleonModel::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    scan_available = true;
    scan_buffer_.push(msg);
}

void NapoleonModel::getLatestScanData()
{

    if(scan_available)
    {
        while(!scan_buffer_.empty())
        {
            scan = scan_buffer_.front();

            // - - - - - - - - - - - - - - - - - -
            // Determine absolute laser pose based on TF

            try
            {
                tf::StampedTransform t_sensor_pose;
                tf_listener_->lookupTransform("map", scan->header.frame_id, scan->header.stamp, t_sensor_pose);
                scan_buffer_.pop();

                tf::Quaternion q = t_sensor_pose.getRotation(); //( t_sensor_pose.getRotation().x, t_sensor_pose.getRotation().y, t_sensor_pose.getRotation().z, t_sensor_pose.getRotation().w );
                tf::Matrix3x3 matrix ( q );
                double rollSensor, pitchSensor, yawSensor;
                matrix.getRPY ( rollSensor, pitchSensor, yawSensor );

                double scan_size =  scan->ranges.size();
                laser_meas_points.resize(scan_size);

                for(unsigned int iScan = 0; iScan < scan_size; iScan ++)
                {
                    double angle = yawSensor + scan->angle_min + scan->angle_increment*iScan;
                    laser_meas_points[iScan].x = t_sensor_pose.getOrigin().getX() + scan->ranges[iScan]*cos( angle );
                    laser_meas_points[iScan].y = t_sensor_pose.getOrigin().getY() + scan->ranges[iScan]*sin( angle );
                }
            }
            catch(tf::ExtrapolationException& ex)
            {
                ROS_WARN_STREAM_DELAYED_THROTTLE(10, "ED Laserplugin tracking: " << ex.what());
                try
                {
                    // Now we have to check if the error was an interpolation or extrapolation error
                    // (i.e., the scan is too old or too new, respectively)
                    tf::StampedTransform latest_transform;
                    tf_listener_->lookupTransform("map", scan->header.frame_id, ros::Time(0), latest_transform);

                    if (scan_buffer_.front()->header.stamp > latest_transform.stamp_)
                    {
                        // Scan is too new
                        break;
                    }
                    else
                    {
                        // Otherwise it has to be too old (pop it because we cannot use it anymore)
                        scan_buffer_.pop();
                    }
                }
                catch(tf::TransformException& exc)
                {
                    scan_buffer_.pop();
                }
            }
            catch(tf::TransformException& exc)
            {
                ROS_ERROR_STREAM_DELAYED_THROTTLE(10, "ED Laserplugin tracking: " << exc.what());
                scan_buffer_.pop();
            }
        }
        scan_available = false;
    }
}

void NapoleonModel::overtakeStateMachine(NapoleonPrediction &P, NapoleonObstacle &O, NapoleonAssignment &A)
{

    O.obs_in_ropod_frame_pos = coordGlobalToRopod(O.obs_center_global, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
    O.v_obs_sq = O.current_obstacle.vel.x*O.current_obstacle.vel.x+O.current_obstacle.vel.y*O.current_obstacle.vel.y;
    if ((O.v_obs_sq < V_OBS_OVERTAKE_MAX*V_OBS_OVERTAKE_MAX) && O.obs_in_ropod_frame_pos.x > 0) {
        //disp("Slow obstacle, check if there is space to overtake");
        if (consider_overtaking_current_hallway) {
            rw_p_rear = getPointByID(A.current_hallway_task[0],A.pointlist);
            rw_p_front = getPointByID(A.current_hallway_task[1],A.pointlist);
            cur_obj = current_hallway;
            areaIDs = cur_obj.getPointIDs();
            ind = 0;
            qmax = areaIDs.size();
            for (int q = 0; q < qmax; ++q) {
                if (areaIDs[q].compare(rw_p_rear.id) == 0) {
                    ind = q;
                }
            }
            rotate(areaIDs.begin(), areaIDs.begin() + ind, areaIDs.end());
            lw_p_rear = getPointByID(areaIDs[3],pointlist);
            lw_p_front = getPointByID(areaIDs[2],pointlist);
        } else if (consider_overtaking_next_hallway) {
            rw_p_rear = getPointByID(next_hallway_task[0],pointlist);
            rw_p_front = getPointByID(next_hallway_task[1],pointlist);
            cur_obj = next_hallway;
            areaIDs = cur_obj.getPointIDs();
            ind = 0;
            qmax = areaIDs.size();
            for (int q = 0; q < qmax; ++q) {
                if (areaIDs[q].compare(rw_p_rear.id) == 0) {
                    ind = q;
                }
            }
            rotate(areaIDs.begin(), areaIDs.begin() + ind, areaIDs.end());
            lw_p_rear = getPointByID(areaIDs[3],pointlist);
            lw_p_front = getPointByID(areaIDs[2],pointlist);
        }

        space_left = distToSegment(obs_center_global,lw_p_rear,lw_p_front);
        space_right = distToSegment(obs_center_global,rw_p_rear,rw_p_front);
        if (current_obstacle.width > current_obstacle.depth) {
            space_left = space_left-current_obstacle.width/2;
            space_right = space_right-current_obstacle.width/2;
        } else {
            space_left = space_left-current_obstacle.depth/2;
            space_right = space_right-current_obstacle.depth/2;
        }

        if (space_right > TUBE_WIDTH_C) {
            if (j == 1) {
                ROS_INFO("No overtake necessary, passing on right should be possible");
            }
        } else if (space_right > 2*(SIZE_SIDE+ENV_TCTW_SIZE)+OBS_AVOID_MARGIN) {
            // Same state, but change tube width so ropod will
            // fit through space right
            pred_tube_width[j] = space_right;
            if (j == 1) {
                ROS_INFO("No overtake necessary, but tube size scaled down");
            }
        } else if (space_left > 2*(SIZE_SIDE+ENV_TRNS_SIZE)+ENV_TCTW_SIZE) {
            if (j == 1) {
                ROS_INFO("Can overtake on left side, there should be enough space there");
            }
            // Start overtake
            if (space_left < TUBE_WIDTH_C) {
                if (obs_in_ropod_frame_pos.x < MIN_DIST_TO_OVERTAKE && abs(obs_in_ropod_frame_pos.y) < MIN_DIST_TO_OVERTAKE) {
                    ROS_INFO("Tight overtake");
                    pred_state[j] = TIGHT_OVERTAKE;
                    //current_to_overtake_obs = to_overtake_obs;
                    if (consider_overtaking_next_hallway) {
                        u = u + delta_assignment_on_overtake; // Cesar TODO-> check if this works (instead of +2 with no consecutive hallways)!
                        update_assignment = true;
                    }
                    pred_tube_width[j] = 2*(SIZE_SIDE+ENV_TCTW_SIZE+ENV_TRNS_SIZE);
                }
            } else {
                if (obs_in_ropod_frame_pos.x < MIN_DIST_TO_OVERTAKE && abs(obs_in_ropod_frame_pos.y) < MIN_DIST_TO_OVERTAKE) {
                    ROS_INFO("Spacious overtake");
                    pred_state[j] = SPACIOUS_OVERTAKE;
                    //current_to_overtake_obs = to_overtake_obs;
                    if (consider_overtaking_next_hallway) {
                        u = u + delta_assignment_on_overtake; // Cesar TODO-> check if this works (instead of +2 with no consecutive hallways)!
                        update_assignment = true;
                    }
                    if (current_obstacle.width > current_obstacle.depth) {
                        shift_wall = space_right+current_obstacle.width+OBS_AVOID_MARGIN;
                    } else {
                        shift_wall = space_right+current_obstacle.depth+OBS_AVOID_MARGIN;
                    }
                    pred_tube_width[j] = TUBE_WIDTH_C;
                }
            }
        } else {
            if (j == 1) {
                ROS_INFO("No overtake possible, stuck behind this obstacle");
            }
        }
    }

}

void NapoleonModel::updatePosition(NapoleonPrediction &P){
    // AMCL data (around 3Hz) - update to AMCL data if new AMCL pose received
    // otherwise make a guess
    if (this_amcl_x == prev_amcl_x && this_amcl_y == prev_amcl_y && this_amcl_theta == prev_amcl_theta) {
        // No AMCL update, so estimate initial values for prediction
        ropod_x = P.pred_x_ropod[1];
        ropod_y = P.pred_y_ropod[1];
        ropod_theta = P.pred_plan_theta[1];
    } else {
        // Set latest values from AMCL to previous AMCL values for next iteration
        // And take AMCL values as initial for the prediction
        prev_amcl_x = this_amcl_x;
        prev_amcl_y = this_amcl_y;
        prev_amcl_theta = this_amcl_theta;
        ropod_x = this_amcl_x;
        ropod_y = this_amcl_y;
        ropod_theta = this_amcl_theta;
    }
}

void NapoleonModel::updateControlVelocity(NapoleonPrediction &P){
    control_v = P.pred_v_ropod[0]+P.pred_accel[1]*1/F_PLANNER;
}
