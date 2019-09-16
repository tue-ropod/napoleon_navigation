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
        if (P.consider_overtaking_current_hallway) {
            P.rw_p_rear = getPointByID(A.current_hallway_task[0],A.pointlist);
            P.rw_p_front = getPointByID(A.current_hallway_task[1],A.pointlist);
            A.cur_obj = A.current_hallway;
            A.areaIDs = A.cur_obj.getPointIDs();
            P.ind = 0;
            P.qmax = A.areaIDs.size();
            for (int q = 0; q < P.qmax; ++q) {
                if (A.areaIDs[q].compare(P.rw_p_rear.id) == 0) {
                    P.ind = q;
                }
            }
            rotate(A.areaIDs.begin(), A.areaIDs.begin() + P.ind, A.areaIDs.end());
            P.lw_p_rear = getPointByID(A.areaIDs[3],A.pointlist);
            P.lw_p_front = getPointByID(A.areaIDs[2],A.pointlist);
        } else if (P.consider_overtaking_next_hallway) {
            P.rw_p_rear = getPointByID(A.next_hallway_task[0],A.pointlist);
            P.rw_p_front = getPointByID(A.next_hallway_task[1],A.pointlist);
            A.cur_obj = A.next_hallway;
            A.areaIDs = A.cur_obj.getPointIDs();
            P.ind = 0;
            P.qmax = A.areaIDs.size();
            for (int q = 0; q < P.qmax; ++q) {
                if (A.areaIDs[q].compare(P.rw_p_rear.id) == 0) {
                    P.ind = q;
                }
            }
            rotate(A.areaIDs.begin(), A.areaIDs.begin() + P.ind, A.areaIDs.end());
            P.lw_p_rear = getPointByID(A.areaIDs[3],A.pointlist);
            P.lw_p_front = getPointByID(A.areaIDs[2],A.pointlist);
        }

        P.space_left = distToSegment(O.obs_center_global,P.lw_p_rear,P.lw_p_front);
        P.space_right = distToSegment(O.obs_center_global,P.rw_p_rear,P.rw_p_front);
        if (O.current_obstacle.width > O.current_obstacle.depth) {
            P.space_left = P.space_left-O.current_obstacle.width/2;
            P.space_right = P.space_right-O.current_obstacle.width/2;
        } else {
            P.space_left = P.space_left-O.current_obstacle.depth/2;
            P.space_right = P.space_right-O.current_obstacle.depth/2;
        }

        if (P.space_right > TUBE_WIDTH_C) {
            if (P.j == 1) {
                ROS_INFO("No overtake necessary, passing on right should be possible");
            }
        } else if (P.space_right > 2*(SIZE_SIDE+ENV_TCTW_SIZE)+OBS_AVOID_MARGIN) {
            // Same state, but change tube width so ropod will
            // fit through space right
            P.pred_tube_width[P.j] = P.space_right;
            if (P.j == 1) {
                ROS_INFO("No overtake necessary, but tube size scaled down");
            }
        } else if (P.space_left > 2*(SIZE_SIDE+ENV_TRNS_SIZE)+ENV_TCTW_SIZE) {
            if (P.j == 1) {
                ROS_INFO("Can overtake on left side, there should be enough space there");
            }
            // Start overtake
            if (P.space_left < TUBE_WIDTH_C) {
                if (O.obs_in_ropod_frame_pos.x < MIN_DIST_TO_OVERTAKE && abs(O.obs_in_ropod_frame_pos.y) < MIN_DIST_TO_OVERTAKE) {
                    ROS_INFO("Tight overtake");
                    P.pred_state[P.j] = TIGHT_OVERTAKE;
                    //current_to_overtake_obs = to_overtake_obs;
                    if (P.consider_overtaking_next_hallway) {
                        P.u = P.u + P.delta_assignment_on_overtake; // Cesar TODO-> check if this works (instead of +2 with no consecutive hallways)!
                        A.update_assignment = true;
                    }
                    P.pred_tube_width[P.j] = 2*(SIZE_SIDE+ENV_TCTW_SIZE+ENV_TRNS_SIZE);
                }
            } else {
                if (O.obs_in_ropod_frame_pos.x < MIN_DIST_TO_OVERTAKE && abs(O.obs_in_ropod_frame_pos.y) < MIN_DIST_TO_OVERTAKE) {
                    ROS_INFO("Spacious overtake");
                    P.pred_state[P.j] = SPACIOUS_OVERTAKE;
                    //current_to_overtake_obs = to_overtake_obs;
                    if (P.consider_overtaking_next_hallway) {
                        P.u = P.u + P.delta_assignment_on_overtake; // Cesar TODO-> check if this works (instead of +2 with no consecutive hallways)!
                        A.update_assignment = true;
                    }
                    if (O.current_obstacle.width > O.current_obstacle.depth) {
                        P.shift_wall = P.space_right+O.current_obstacle.width+OBS_AVOID_MARGIN;
                    } else {
                        P.shift_wall = P.space_right+O.current_obstacle.depth+OBS_AVOID_MARGIN;
                    }
                    P.pred_tube_width[P.j] = TUBE_WIDTH_C;
                }
            }
        } else {
            if (P.j == 1) {
                ROS_INFO("No overtake possible, stuck behind this obstacle");
            }
        }
    }

}

void NapoleonModel::computeSteeringAndVelocity(ros::Publisher &wallmarker_pub, NapoleonAssignment &A, NapoleonPrediction &P, NapoleonVisualization &V)
{
    // Perform the appropriate action according to finite state machine
    if (P.pred_state[P.j] == CRUSING || P.pred_state[P.j] == GOING_STRAIGHT_ON_INTERSECTION || P.pred_state[P.j] == TIGHT_OVERTAKE || P.pred_state[P.j] == SPACIOUS_OVERTAKE) {
        // disp([num2str[j],' - Ropod is now cruising']);
        if (P.pred_state[P.j] == CRUSING) {   // Cruising up
            if (P.update_state_points) {
                A.point_rear = getPointByID(A.task1[0],A.pointlist);
                A.point_front = getPointByID(A.task1[1],A.pointlist);
                //if(j==1 && u == ka_max-1) ROS_INFO("Cruising Point rear: %s, Point front: %s", point_rear.id.c_str(), point_front.id.c_str());
                A.glob_wallpoint_front.x = A.point_front.x;
                A.glob_wallpoint_front.y = A.point_front.y;
                A.glob_wallpoint_rear.x = A.point_rear.x;
                A.glob_wallpoint_rear.y = A.point_rear.y;
            }
            v_des = V_CRUISING;
        } else if (P.pred_state[P.j] == GOING_STRAIGHT_ON_INTERSECTION) { // Straight on inter
            if (P.update_state_points) {
                A.point_rear = getPointByID(A.task2[0],A.pointlist);
                A.point_front = getPointByID(A.task2[1],A.pointlist);
                A.glob_wallpoint_front.x = A.point_front.x;
                A.glob_wallpoint_front.y = A.point_front.y;
                A.glob_wallpoint_rear.x = A.point_rear.x;
                A.glob_wallpoint_rear.y = A.point_rear.y;
            }
            v_des = V_INTER_ACC;
        } else if (P.pred_state[P.j] == TIGHT_OVERTAKE) { // Tight overtake
            P.rw_p_rear = getPointByID(A.task1[0],A.pointlist);
            A.cur_obj = getAreaByID(A.area1ID,A.arealist);
            A.areaIDs = A.cur_obj.getPointIDs();
            P.ind = 0;
            P.qmax = A.areaIDs.size();
            for (int q = 0; q < P.qmax; ++q) {
                if (A.areaIDs[q].compare(P.rw_p_rear.id) == 0) {
                    P.ind = q;
                }
            }
            rotate(A.areaIDs.begin(), A.areaIDs.begin() + P.ind, A.areaIDs.end());
            P.lw_p_rear = getPointByID(A.areaIDs[3],A.pointlist);
            P.lw_p_front = getPointByID(A.areaIDs[2],A.pointlist);
            P.wallang = atan2(P.lw_p_front.y-P.lw_p_rear.y,P.lw_p_front.x-P.lw_p_rear.x);
            //glob_wallpoint_front = [lw_front.x, lw_front.y]+pred_tube_width[j]*[cos(wallang-M_PI/2), sin(wallang-M_PI/2)];
            A.glob_wallpoint_front.x = P.lw_p_front.x+P.pred_tube_width[P.j]*cos(P.wallang-M_PI/2);
            A.glob_wallpoint_front.y = P.lw_p_front.y+P.pred_tube_width[P.j]*sin(P.wallang-M_PI/2);
            //glob_wallpoint_rear = [lw_rear.x, lw_rear.y]+pred_tube_width[j]*[cos(wallang-M_PI/2), sin(wallang-M_PI/2)];
            A.glob_wallpoint_rear.x = P.lw_p_rear.x+P.pred_tube_width[P.j]*cos(P.wallang-M_PI/2);
            A.glob_wallpoint_rear.y = P.lw_p_rear.y+P.pred_tube_width[P.j]*sin(P.wallang-M_PI/2);
            v_des = V_OVERTAKE;
        } else if (P.pred_state[P.j] == SPACIOUS_OVERTAKE) { // Spacious overtake
            A.point_rear = getPointByID(A.task1[0],A.pointlist);
            A.point_front = getPointByID(A.task1[1],A.pointlist);
            P.wallang = atan2(A.point_front.y-A.point_rear.y,A.point_front.x-A.point_rear.x);
            //glob_wallpoint_front = [point_front.x, point_front.y]+shift_wall*[cos(wallang+M_PI/2), sin(wallang+M_PI/2)];
            //glob_wallpoint_rear = [point_rear.x, point_rear.y]+shift_wall*[cos(wallang+M_PI/2), sin(wallang+M_PI/2)];
            A.glob_wallpoint_front.x = A.point_front.x+P.shift_wall*cos(P.wallang+M_PI/2);
            A.glob_wallpoint_front.y = A.point_front.y+P.shift_wall*sin(P.wallang+M_PI/2);
            A.glob_wallpoint_rear.x = A.point_rear.x+P.shift_wall*cos(P.wallang+M_PI/2);
            A.glob_wallpoint_rear.y = A.point_rear.y+P.shift_wall*sin(P.wallang+M_PI/2);
            v_des = V_OVERTAKE;
        }

        A.local_wallpoint_front = coordGlobalToRopod(A.glob_wallpoint_front, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        A.local_wallpoint_rear = coordGlobalToRopod(A.glob_wallpoint_rear, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        V.showWallPoints(A.local_wallpoint_front, A.local_wallpoint_rear, wallmarker_pub);
        P.pred_phi_des[P.j] = getSteering(A.local_wallpoint_front, A.local_wallpoint_rear, P.pred_tube_width[P.j]);
    } else if (P.pred_state[P.j] == ENTRY_BEFORE_TURN_ON_INTERSECTION || P.pred_state[P.j] == ENTRY_BEFORE_GOING_STRAIGHT_ON_INTERSECTION) {
        // disp([num2str[j],' - Ropod is now in entry']);
        if (P.update_state_points) {
            A.point_rear = getPointByID(A.task1[0],A.pointlist);
            A.point_front = getPointByID(A.task1[1],A.pointlist);
            A.glob_wallpoint_front.x = A.point_front.x;
            A.glob_wallpoint_front.y = A.point_front.y;
            A.glob_wallpoint_rear.x = A.point_rear.x;
            A.glob_wallpoint_rear.y = A.point_rear.y;
        }
        A.local_wallpoint_front = coordGlobalToRopod(A.glob_wallpoint_front, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        A.local_wallpoint_rear = coordGlobalToRopod(A.glob_wallpoint_rear, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        V.showWallPoints(A.local_wallpoint_front, A.local_wallpoint_rear, wallmarker_pub);
        P.pred_phi_des[P.j] = getSteering(A.local_wallpoint_front, A.local_wallpoint_rear, P.pred_tube_width[P.j]);
        v_des = V_ENTRY;

    } else if (P.pred_state[P.j] == ACCELERATE_ON_INTERSECTION) {
        // disp([num2str[j],' - Ropod is at inter, driving forward']);
        if (P.update_state_points) {
            A.point_rear = getPointByID(A.task1[0],A.pointlist);
            A.point_front = getPointByID(A.task1[1],A.pointlist);
            A.glob_wallpoint_front.x = A.point_front.x;
            A.glob_wallpoint_front.y = A.point_front.y;
            A.glob_wallpoint_rear.x = A.point_rear.x;
            A.glob_wallpoint_rear.y = A.point_rear.y;
        }

        A.local_wallpoint_front = coordGlobalToRopod(A.glob_wallpoint_front, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        A.local_wallpoint_rear = coordGlobalToRopod(A.glob_wallpoint_rear, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        V.showWallPoints(A.local_wallpoint_front, A.local_wallpoint_rear, wallmarker_pub);
        P.pred_phi_des[P.j] = getSteering(A.local_wallpoint_front, A.local_wallpoint_rear, P.pred_tube_width[P.j]);
        v_des = V_INTER_ACC;

        // Monitor if we don't bump into front wall
        if (P.update_state_points) {
            A.wall_front_p0 = getPointByID(A.task2[0],A.pointlist);
            A.wall_front_p1 = getPointByID(A.task2[1],A.pointlist);
            A.global_wall_front_p0.x = A.wall_front_p0.x;
            A.global_wall_front_p0.y = A.wall_front_p0.y;
            A.global_wall_front_p1.x = A.wall_front_p1.x;
            A.global_wall_front_p1.y = A.wall_front_p1.y;
        }

        A.local_wall_front_p0 = coordGlobalToRopod(A.global_wall_front_p0, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        A.local_wall_front_p1 = coordGlobalToRopod(A.global_wall_front_p1, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        if (do_lines_intersect(A.local_front_ropod_dilated_p0, A.local_front_ropod_dilated_p1, A.local_wall_front_p0, A.local_wall_front_p1)) {
            P.pred_state[P.j] = TURNING;
            P.update_state_points = true;
            //disp("Switched early while aligning pivot because too close to front wall");
        }
    } else if (P.pred_state[P.j] == ALIGN_AXIS_AT_INTERSECTION) {
        // disp([num2str[j],' - Ropod is at inter, driving forward']);
        if (P.update_state_points) {
            A.point_rear = getPointByID(A.task1[0],A.pointlist);
            A.point_front = getPointByID(A.task1[1],A.pointlist);
            A.glob_wallpoint_front.x = A.point_front.x;
            A.glob_wallpoint_front.y = A.point_front.y;
            A.glob_wallpoint_rear.x = A.point_rear.x;
            A.glob_wallpoint_rear.y = A.point_rear.y;
        }

        A.local_wallpoint_front = coordGlobalToRopod(A.glob_wallpoint_front, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        A.local_wallpoint_rear = coordGlobalToRopod(A.glob_wallpoint_rear, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        V.showWallPoints(A.local_wallpoint_front, A.local_wallpoint_rear, wallmarker_pub);
        P.pred_phi_des[P.j] = getSteering(A.local_wallpoint_front, A.local_wallpoint_rear, P.pred_tube_width[P.j]);
        v_des = V_INTER_DEC;

        // Monitor if we don't bump into front wall
        if (P.update_state_points) {
            A.wall_front_p0 = getPointByID(A.task2[0],A.pointlist);
            A.wall_front_p1 = getPointByID(A.task2[1],A.pointlist);
            A.global_wall_front_p0.x = A.wall_front_p0.x;
            A.global_wall_front_p0.y = A.wall_front_p0.y;
            A.global_wall_front_p1.x = A.wall_front_p1.x;
            A.global_wall_front_p1.y = A.wall_front_p1.y;
        }

        A.local_wall_front_p0 = coordGlobalToRopod(A.global_wall_front_p0, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        A.local_wall_front_p1 = coordGlobalToRopod(A.global_wall_front_p1, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        if (do_lines_intersect(A.local_front_ropod_dilated_p0, A.local_front_ropod_dilated_p1, A.local_wall_front_p0, A.local_wall_front_p1)) {
            P.pred_state[P.j] = TURNING;
            P.update_state_points = true;
            //disp("Switched early while aligning pivot because too close to front wall");
        }
    } else if (P.pred_state[P.j] == TURNING) {
        // disp([num2str[j],' - Ropod is at inter, taking the turn']);
        if (P.update_state_points) {
            A.point_rear = getPointByID(A.task2[0],A.pointlist);
            A.point_front = getPointByID(A.task2[1],A.pointlist);
            A.point_pivot = getPointByID(A.task2[4],A.pointlist);
            A.glob_wallpoint_front.x = A.point_front.x;
            A.glob_wallpoint_front.y = A.point_front.y;
            A.glob_wallpoint_rear.x = A.point_rear.x;
            A.glob_wallpoint_rear.y = A.point_rear.y;
        }

        P.dir_cw = true;
        if (A.task2[5].compare("left") == 0) {
            //if (strcmp(task2{6},'left')) {
            P.dir_cw = false; // direction 0 = CCW, 1 = CW
        }

        A.local_pivot = coordGlobalToRopod(A.point_pivot, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        if (!A.sharp_corner[P.u+1]) {
            A.local_wallpoint_front = coordGlobalToRopod(A.glob_wallpoint_front, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
            A.local_wallpoint_rear = coordGlobalToRopod(A.glob_wallpoint_rear, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
            //pred_phi_des[j] = getSteeringTurn(ropod_length, size_side, feeler_size_steering, d_ax, dir_cw, local_pivot, local_wallpoint_front, local_wallpoint_rear,  follow_wall_distance, env_tctw_size, env_trns_size_cornering, env_carrot_size);
            P.pred_phi_des[P.j] = getSteeringTurn(A.local_pivot, P.dir_cw, A.local_wallpoint_front, A.local_wallpoint_rear);
        } else {
            //pred_phi_des[j] = getSteeringTurnSharp(pred_x_ropod(j-1), pred_y_ropod(j-1), pred_plan_theta[j-1], size_front_ropod, size_side, feeler_size_steering, d_ax, dir_cw, task2, pointlist, follow_wall_distance, env_tctw_size, env_trns_size_cornering, env_carrot_size);
            P.pred_phi_des[P.j] = getSteeringTurnSharp(P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1], P.dir_cw, A.task2, A.pointlist);
        }

        v_des = V_INTER_TURNING;
    }

    // Wrap to [-pi,pi] domain
    P.pred_phi_des[P.j] = wrapToPi(P.pred_phi_des[P.j]);

    // Saturate steering rate
    if (abs(P.pred_phi_des[P.j]-P.prev_pred_phi_des) > DELTA_DOT_LIMIT/(double)F_PLANNER) {
        //disp("Delta steering too large, steering saturated");
        P.pred_phi_des[P.j] = P.prev_pred_phi_des + sgn(P.pred_phi_des[P.j]-P.prev_pred_phi_des)*DELTA_DOT_LIMIT/(double)F_PLANNER;
        // Decrease vel leads to better corners
        // The velocity is already decreased in the state machine, but this is just a harsh backup
        // pred_steer_rate_saturation[j] = 1;
        if (v_des > V_STEERSATURATION) {
            v_des = V_STEERSATURATION;
        }
        //disp(['In saturation - j: ', num2str(j) ,', Phides: ', num2str(pred_phi_des(j)), ' // Prev phides: ' , num2str(prev_pred_phi_des), ', v_des = ', num2str(v_des)]);
    }
    P.prev_pred_phi_des = P.pred_phi_des[P.j];

    // Applying the v_scale (scales down vel if collision is detected in previous prediction and therefore it failed)
    // And then we calculate the acceleration for the predictions.
    P.v_des_scaled[P.j] = v_des*v_scale;
    P.vel_dif = abs(P.pred_v_ropod_plan[P.j-1]-P.v_des_scaled[P.j]);   // Difference between actual and desired velocity
    if (P.vel_dif < max_delta_v) {
        P.v_new = P.v_des_scaled[P.j];
        P.pred_accel[P.j] = (P.v_des_scaled[P.j]-P.pred_v_ropod_plan[P.j-1])/delta_t;
    } else { // Adapt velocity with maximum acceleration
        P.v_new = P.pred_v_ropod_plan[P.j-1]+sgn(P.v_des_scaled[P.j]-P.pred_v_ropod_plan[P.j-1])*max_delta_v;
        P.pred_accel[P.j] = sgn(P.v_des_scaled[P.j]-P.pred_v_ropod_plan[P.j-1])*A_MAX;
    }
    //ROS_INFO("pred_accel: %f", pred_accel[j]);
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
