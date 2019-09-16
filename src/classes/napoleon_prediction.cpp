#include "napoleon_prediction.h"

void NapoleonPrediction::checkForCollisions(NapoleonModel &M, NapoleonObstacle &O)
{
    /**
     * Check for collision
     * Either for obstacles or virtual walls(to be added)
     * */
    if (O.no_obs > 0) {
        pred_x_obs[j] = pred_x_obs[j-1]+O.current_obstacle.vel.x*TS*F_FSTR;
        pred_y_obs[j] = pred_y_obs[j-1]+O.current_obstacle.vel.y*TS*F_FSTR;
    }

    // Dilated vehicle
    pred_ropod_dil_rb.x = pred_x_rearax[m] + (SIZE_REAR+(OBS_AVOID_MARGIN*M.v_scale))*cos(pred_theta[m]+M_PI) + (SIZE_SIDE+(OBS_AVOID_MARGIN*M.v_scale))*cos(pred_theta[m]-M_PI/2);
    pred_ropod_dil_rb.y = pred_y_rearax[m] + (SIZE_REAR+(OBS_AVOID_MARGIN*M.v_scale))*sin(pred_theta[m]+M_PI) + (SIZE_SIDE+(OBS_AVOID_MARGIN*M.v_scale))*sin(pred_theta[m]-M_PI/2);
    pred_ropod_dil_lb.x = pred_x_rearax[m] + (SIZE_REAR+(OBS_AVOID_MARGIN*M.v_scale))*cos(pred_theta[m]+M_PI) + (SIZE_SIDE+(OBS_AVOID_MARGIN*M.v_scale))*cos(pred_theta[m]+M_PI/2);
    pred_ropod_dil_lb.y = pred_y_rearax[m] + (SIZE_REAR+(OBS_AVOID_MARGIN*M.v_scale))*sin(pred_theta[m]+M_PI) + (SIZE_SIDE+(OBS_AVOID_MARGIN*M.v_scale))*sin(pred_theta[m]+M_PI/2);
    pred_ropod_dil_lt.x = pred_x_rearax[m] + (SIZE_FRONT_RAX+(OBS_AVOID_MARGIN*M.v_scale))*cos(pred_theta[m]) + (SIZE_SIDE+(OBS_AVOID_MARGIN*M.v_scale))*cos(pred_theta[m]+M_PI/2);
    pred_ropod_dil_lt.y = pred_y_rearax[m] + (SIZE_FRONT_RAX+(OBS_AVOID_MARGIN*M.v_scale))*sin(pred_theta[m]) + (SIZE_SIDE+(OBS_AVOID_MARGIN*M.v_scale))*sin(pred_theta[m]+M_PI/2);
    pred_ropod_dil_rt.x = pred_x_rearax[m] + (SIZE_FRONT_RAX+(OBS_AVOID_MARGIN*M.v_scale))*cos(pred_theta[m]) + (SIZE_SIDE+(OBS_AVOID_MARGIN*M.v_scale))*cos(pred_theta[m]-M_PI/2);
    pred_ropod_dil_rt.y = pred_y_rearax[m] + (SIZE_FRONT_RAX+(OBS_AVOID_MARGIN*M.v_scale))*sin(pred_theta[m]) + (SIZE_SIDE+(OBS_AVOID_MARGIN*M.v_scale))*sin(pred_theta[m]-M_PI/2);

    M.ropod_colliding_obs = false;
    pred_ropod_colliding_obs[j] = false;
    // Obstacle detection (crappy implementation in C++)
    if (t_pred[m] < T_PRED_OBS_COLLISION && O.no_obs > 0) {
        obslt.y = pred_y_obs[j]+O.current_obstacle.width/2*sin(O.obs_theta)+O.current_obstacle.depth/2*cos(O.obs_theta);
        obslt.x = pred_x_obs[j]+O.current_obstacle.width/2*cos(O.obs_theta)-O.current_obstacle.depth/2*sin(O.obs_theta);
        obsrt.x = pred_x_obs[j]+O.current_obstacle.width/2*cos(O.obs_theta)+O.current_obstacle.depth/2*sin(O.obs_theta);
        obsrt.y = pred_y_obs[j]+O.current_obstacle.width/2*sin(O.obs_theta)-O.current_obstacle.depth/2*cos(O.obs_theta);
        obslb.x = pred_x_obs[j]-O.current_obstacle.width/2*cos(O.obs_theta)-O.current_obstacle.depth/2*sin(O.obs_theta);
        obslb.y = pred_y_obs[j]-O.current_obstacle.width/2*sin(O.obs_theta)+O.current_obstacle.depth/2*cos(O.obs_theta);
        obsrb.x = pred_x_obs[j]-O.current_obstacle.width/2*cos(O.obs_theta)+O.current_obstacle.depth/2*sin(O.obs_theta);
        obsrb.y = pred_y_obs[j]-O.current_obstacle.width/2*sin(O.obs_theta)-O.current_obstacle.depth/2*cos(O.obs_theta);
        pred_ropod_colliding_obs[j] = do_shapes_overlap(pred_ropod_dil_rb, pred_ropod_dil_lb, pred_ropod_dil_lt, pred_ropod_dil_rt, obsrt, obslt, obslb, obsrb);
        M.ropod_colliding_obs = pred_ropod_colliding_obs[j];
    }

    // TODO: Add here also check with raw laser (for instance of non-associated objects) data or costmap
    if (t_pred[m] < T_PRED_OBS_COLLISION ) // TODO: Condition should be smarter. For instance based on time/distance to stop? . Also the robot should stop at a minimum distance in front. for now we based it on time
    {
        // Here check for collision with laser data
        // Create Area and use contain method with the laser data.
        for(unsigned int iScan = 0; iScan < M.laser_meas_points.size(); iScan++)
        {
            AreaQuad robot_footprint(pred_ropod_dil_rb, pred_ropod_dil_lb, pred_ropod_dil_lt, pred_ropod_dil_rt);
            Point laser_point(M.laser_meas_points[iScan].x, M.laser_meas_points[iScan].y);
            pred_ropod_colliding_obs[j] = robot_footprint.contains(laser_point);
            M.ropod_colliding_obs = pred_ropod_colliding_obs[j];
            if(M.ropod_colliding_obs)
                break;
        }
    }


    // Predict intersection times (left out for now)
    M.ropod_colliding_wall = false;

    // TODO: Finish next line to add checking collision with walls!

    // Predict intersection with walls
    // if ((u < ka_max-1) && update_assignment) {
    //     walls = getWalls(assignment[u],assignment[u+1],assignment[u+2],arealist);
    // }
    // if (t_pred[m] < T_PRED_WALL_COLLISION) {
    //     pred_ropod_wall_collision(j) = 0;
    //     for wallidx = 1:size(walls,1)
    //         wall = walls(wallidx,:);
    //         wp0 = getPointByID(wall{1},pointlist);
    //         wp1 = getPointByID(wall{2},pointlist);
    //         ropod = [pred_ropod_rb(j,:); pred_ropod_lb(j,:); pred_ropod_lt(j,:); pred_ropod_rt(j,:)];
    //         if does_line_intersect_shape(wp0, wp1, ropod)
    //             pred_ropod_wall_collision(j) = 1;
    //             // disp([num2str(t_pred(m)),': Oh noes I collided wiv a wall']);
    //         end
    //     end
    // }
}

void NapoleonPrediction::simulateRobotDuringCurrentPredictionStep()
{
    /**
     * Simulate robot during current prediction step.
     * Simulations are stacked over the different prediction steps.
     * */
    m_prev = m;
    t_pred_prev = t_pred[m];
    t_pred_j[j] = t_pred_prev+F_FSTR*TS;

    // Simulate ropod motion with current plan (Simulation is done faster than controller sample time)
    for (int q = 1; q <= F_FSTR; ++q) { // q = [1, 2, ..., F_FSTR]
        m = m_prev+q;           // Current iteration
        t_pred[m] = t_pred_prev+q*TS;

        pred_phi[m] = (1-lpf)*pred_phi[m-1]+lpf*pred_phi_des[j];
        pred_theta[m] = wrapToPi(pred_theta[m-1]+pred_thetadot[m-1]*TS);
        pred_v_ropod[m] = pred_v_ropod[m-1]+pred_accel[j]*TS;

        pred_xdot[m] = pred_v_ropod[m]*cos(pred_phi[m])*cos(pred_theta[m]);
        pred_ydot[m] = pred_v_ropod[m]*cos(pred_phi[m])*sin(pred_theta[m]);
        pred_thetadot[m] = pred_v_ropod[m]*1/D_AX*sin(pred_phi[m]);

        pred_x_rearax[m] = pred_x_rearax[m-1]+pred_xdot[m]*TS;
        pred_y_rearax[m] = pred_y_rearax[m-1]+pred_ydot[m]*TS;
    }
}


void NapoleonPrediction::initialize(NapoleonModel &M, NapoleonObstacle &O)
{
    M.v_scale = V_SCALE_OPTIONS[k];
    k++;
    m = 0;
    j = 0;
    t_pred[m] = 0;
    t_pred_j[j] = 0;

    // Initialize prediction with latest sim values
    pred_phi[0] = M.odom_phi_local; // On ropod
    pred_theta[0] = M.ropod_theta;
    pred_v_ropod[0] = M.control_v;
    if (abs(M.odom_vropod_global-M.control_v) > 0.5) {
        ROS_INFO("Difference between control and actual velocity > 0.5, correcting now.");
        pred_v_ropod[0] = M.odom_vropod_global;
    }
    pred_xdot[0] = pred_v_ropod[0]*cos(pred_phi[0])*cos(M.ropod_theta);   // xdot of rearaxle in global frame
    pred_ydot[0] = pred_v_ropod[0]*cos(pred_phi[0])*sin(M.ropod_theta);   // ydot of rearaxle in global frame
    pred_thetadot[0] = pred_v_ropod[0]*1/D_AX*sin(pred_phi[0]);
    pred_x_rearax[0] = M.ropod_x-D_AX*cos(M.ropod_theta);
    pred_y_rearax[0] = M.ropod_y-D_AX*sin(M.ropod_theta);
    pred_xy_ropod[0].x = M.ropod_x;
    pred_xy_ropod[0].y = M.ropod_y;
    prev_pred_phi_des = prev_sim_phi_des;
    pred_phi_des[0] = prev_pred_phi_des;
    pred_tube_width[0] = prev_sim_tube_width;
    pred_plan_theta[0] = M.ropod_theta;
    pred_v_ropod_plan[0] = pred_v_ropod[0];
    pred_state[0] = prev_sim_state;
    pred_task_counter[0] = prev_sim_task_counter;
    pred_x_obs[0] = O.current_obstacle.pose.position.x;
    pred_y_obs[0] = O.current_obstacle.pose.position.y;
}

void NapoleonPrediction::update(NapoleonAssignment &A, NapoleonModel &M, NapoleonObstacle &O, NapoleonVisualization &V){

    // Prediction
    while (t_pred[m] < T_MIN_PRED) {
        j = j+1;

        // j and m counter are initialized at 0 instead of 1 in Matlab so we dont have to change their indices
        consider_overtaking_current_hallway = false;
        consider_overtaking_next_hallway = false;
        pred_tube_width[j] = pred_tube_width[j-1];  // Assume same as previous, might change later

        A.updateAreasAndFeatures();

        /**
         * Check whether robot is entrying a new area
         * */
        // Original implementation was checking for overlap between ropod shape and entry shape,
        // maybe change later if this implementation causes strange behavior
        //if(j==1)printf("Areas type curr next: %s, %s\n",curr_area.type.c_str(),next_area.type.c_str());
        pred_ropod_on_entry_inter[j] = false;
        pred_ropod_on_entry_hall[j] = false;
        if (A.curr_area.type == "hallway" && A.next_area.type == "inter")
        {
            A.point_front = getPointByID(A.task1[1],A.pointlist);
            A.local_wallpoint_front = coordGlobalToRopod(A.point_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
            pred_ropod_on_entry_inter[j] = A.local_wallpoint_front.x < ENTRY_LENGTH;

        }
        else if (A.area1ID!=A.area2ID && A.curr_area.type == "hallway" && A.next_area.type == "hallway")
        {
            double walls_angle = getAngleBetweenHallways(A.task1, A.task2, A.pointlist);
            double distance_to_switch_halls;
            if(walls_angle > 0) // Concave, switch early
                distance_to_switch_halls = SIZE_FRONT_ROPOD+1.0;
            else // Convex, switch close to turning axis
                distance_to_switch_halls = -0.5*D_AX;

            A.point_front = getPointByID(A.task1[1],A.pointlist);
            A.local_wallpoint_front = coordGlobalToRopod(A.point_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
            pred_ropod_on_entry_hall[j] = A.local_wallpoint_front.x < distance_to_switch_halls;
        }

        update_state_points = true; // Initially true, might change to false when not necessary
        prevstate = j-1;

        /**
         * Based on predicted position, state, areas, features, update state and task
         * */
        A.updateStateAndTask();
        /**
         * Check wether an overtake should be considered at all
         * */
        considerOvertaking(A, O);

        /**
         * Call overtake state machine when needed
         * */
        if ((pred_state[prevstate] == M.CRUSING && consider_overtaking_current_hallway) || (consider_overtaking_next_hallway && (pred_state[prevstate] == M.TURNING || pred_state[prevstate] == M.GOING_STRAIGHT_ON_INTERSECTION)))
        {
            M.overtakeStateMachine();
        }
        else if (!consider_overtaking_current_hallway && !consider_overtaking_next_hallway)
        {
            pred_tube_width[j] = TUBE_WIDTH_C;
        }

        /**
         * Compute steering and default forward acceleration based on computed state and local features
         * */
        M.computeSteeringAndVelocity();

        /**
         * Simulate robot during current prediction step.
         * Simulations are stacked over the different prediction steps.
         * */
        simulateRobotDuringCurrentPredictionStep();

        /**
         * Check for collision
         * Either for obstacles or virtual walls(to be added)
         * */
        checkForCollisions(M, O);
        if(M.ropod_colliding_obs || M.ropod_colliding_wall) // Do not execute current plan when collision is precited
            break;


        V.visualizeRopodMarkers();

        // Update positions used to make the prediction plan with
        // Cesar-> TODO: In theory D_AX should be replaced by ROPOD_TO_AX, but it brings issues.
        //               Or rename predictions not to ropod but to steering point?
        pred_x_ropod[j] = pred_x_rearax[m]+D_AX*cos(pred_theta[m]);
        pred_y_ropod[j] = pred_y_rearax[m]+D_AX*sin(pred_theta[m]);
        pred_xy_ropod[j].x = pred_x_ropod[j];
        pred_xy_ropod[j].y = pred_y_ropod[j];
        pred_v_ropod_plan[j] = pred_v_ropod[m];

        if (j == 1) {
            //ROS_INFO("v[0] = %f, a[1] = %f, ts = %f, v[1] = %f", pred_v_ropod[0], pred_accel[j], TS, pred_v_ropod_plan[j]);
            // ROS_INFO("v[0] = %f, a[1] = %f, deltav = %f, v[1] = %f", pred_v_ropod[0], pred_accel[j], pred_accel[j]*F_PLANNER, pred_v_ropod[0]+pred_accel[1]*F_PLANNER);
        }
        pred_plan_theta[j] = pred_theta[m];
        //ROS_INFO("j: %d / State: %d / Time: %f / Phi: %f / V_des: %f", j, pred_state[j], t_pred_j[j], pred_phi_des[j], pred_v_ropod[j]);

    } // endwhile prediction
}

void NapoleonPrediction::considerOvertaking(NapoleonAssignment &A, NapoleonObstacle &O)
{

    // Consider overtaking.
    // Cesar -> TODO: Because of the changes to support consecutive hallway areas, this part still needs to be adapted
    // Cesar -> TODO: The contains function should consider not only the center but all corners. Also how to deal with multiple obstacles?
    if (u < A.ka_max-1 && O.no_obs > 0) {
        if (A.update_assignment) {
            if(A.curr_area.type == "hallway"){
                A.current_hallway = A.curr_area;
                A.current_hallway_task = A.task1;
                if (A.current_hallway.contains(O.obs_center_global)) {
                    consider_overtaking_current_hallway = true;
                    // This strategy only allows to overtake 1 obstacle
                }
            }else if(A.next_area.type == "hallway")
            {
                A.next_hallway = A.next_area;
                A.next_hallway_task = A.task2;
                delta_assignment_on_overtake = 1;
                if (A.next_hallway.contains(O.obs_center_global)) {
                    consider_overtaking_next_hallway = true;
                    // This strategy only allows to overtake 1 obstacle
                }

            }
            else if(A.next_second_area.type == "hallway")
            {
                A.next_hallway = A.next_second_area;
                A.next_hallway_task = A.task3;
                delta_assignment_on_overtake = 2;
                if (A.next_hallway.contains(O.obs_center_global)) {
                    consider_overtaking_next_hallway = true;
                    // This strategy only allows to overtake 1 obstacle
                }

            }
        }
    }
}

bool NapoleonPrediction::checkIfFinished(NapoleonAssignment &A){
    bool ropod_reached_target = true;
    if (prev_sim_task_counter == (A.ka_max-1) ) {

        A.point_front = getPointByID(A.task1[1],A.pointlist);
        A.local_wallpoint_front = coordGlobalToRopod(A.point_front, pred_xy_ropod[1], pred_plan_theta[1]);
        if(A.local_wallpoint_front.x < REACHEDTARGETTRESHOLD)
        {
            ropod_reached_target = true;
            ROS_INFO("Ropod has reached its target, yay!");
        }
    }
    return ropod_reached_target;
}

void NapoleonPrediction::step(){
    prev_sim_state = pred_state[1];
    prev_sim_task_counter = pred_task_counter[1];
    prev_sim_phi_des = pred_phi_des[1];
    prev_sim_tube_width = pred_tube_width[1];
}
