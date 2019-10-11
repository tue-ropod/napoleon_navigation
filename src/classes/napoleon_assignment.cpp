#include "napoleon_assignment.h"

void NapoleonAssignment::initializeAssignment(std::vector<ropod_ros_msgs::Area> &plan)
{
    planner_areas = plan;
    int intermediate_area_id_counter = 10000;
    for (int i = 0; i < planner_areas.size(); i++)
    {
        for (int j = 0; j < planner_areas[i].sub_areas.size(); j++)
        {
            for (int k = 0; k < planner_areas[i].sub_areas[j].geometry.vertices.size(); k++)
            {
                pointlist.push_back(PointID(planner_areas[i].sub_areas[j].geometry.vertices[k].x,
                                             planner_areas[i].sub_areas[j].geometry.vertices[k].y,
                                             std::to_string(planner_areas[i].sub_areas[j].geometry.vertices[k].id)));
            }

            int points_size = pointlist.size();
            std::string sub_area_type = planner_areas[i].type;

            if (sub_area_type == "junction")
            {
                sub_area_type = "inter";
            }
            else
            {
                sub_area_type = "hallway";
            }

            int sub_area_id = 0;
            if (points_size >= 4)
            {

                if(planner_areas[i].sub_areas[j].id != "")
                {
                    sub_area_id = std::stoi(planner_areas[i].sub_areas[j].id.c_str());
                }
                else
                {
                    sub_area_id = intermediate_area_id_counter;
                    intermediate_area_id_counter = intermediate_area_id_counter + 1;
                }

                arealist.push_back(AreaQuadID(pointlist[points_size-4],
                                               pointlist[points_size-3],
                                               pointlist[points_size-2],
                                               pointlist[points_size-1],
                                               sub_area_id,
                                               sub_area_type));

            }
            else
            {
                ROS_ERROR("AREA WITH LESS THAN 4 POINTS");
            }
            ROS_INFO("Sub area id: %d | Sub area type: %s", sub_area_id, sub_area_type.c_str());
            assignment.push_back(sub_area_id);
        }
    }

    // update area value
    ka_max = assignment.size();  // Assignment length

    cur_obj = getAreaByID(assignment[0],arealist);

    current_hallway = getAreaByID(assignment[0],arealist); // TODO: This assumes we start in a hallway, which could be not the case when stuck in an intersection
    for (int arID = 1; arID < ka_max; arID++)
    {
        if (arealist[arID].type == "hallway")
        {
            printf("AreaNumber initial next hallway: %d\n",arID);
            next_hallway = arealist[arID];
            break;
        }
    }

    ROS_INFO("Compute assigment nodes from assigment area vector:");

    std::vector<std::string> OBJ1TASK, OBJ2TASK;
    AreaQuadID OBJ1 = getAreaByID(assignment[0],arealist);
    AreaQuadID OBJ2 = getAreaByID(assignment[1],arealist);
    AreaQuadID OBJ3 = getAreaByID(assignment[2],arealist);
    int obj2tasklen;

    for (int ka = 0; ka < ka_max; ka = ka+1) {
        AreaQuadID curr_OBJ = getAreaByID(assignment[ka],arealist);
        std::vector<string> area_names;

        if(curr_OBJ.type=="hallway")
        {
            printf("Hallway assignment: ");
            OBJ1 = getAreaByID(assignment[ka],arealist);
            OBJ2 = getAreaByID(assignment[ka+1],arealist);
            OBJ3 = getAreaByID(assignment[ka+2],arealist);
            if (ka < (ka_max-1) )
            {
                OBJ1TASK = getWallPointsTowardsB(OBJ1,OBJ2);
            }
            else
            {
                OBJ1TASK = getWallPointsAwayFromB(OBJ1,getAreaByID(assignment[ka-1],arealist));
            }
            printstringvec(OBJ1TASK);
            area_names.push_back(OBJ1TASK[0]);
            area_names.push_back(OBJ1TASK[1]);
            area_names.push_back("");
            area_names.push_back("");
            area_names.push_back("");
            area_names.push_back("");
            OBJ_X_TASK.push_back(area_names);
        }
        else if(curr_OBJ.type=="inter")
        {
            printf("Intersection assignment: ");
            OBJ1 = getAreaByID(assignment[ka-1],arealist);
            OBJ2 = getAreaByID(assignment[ka],arealist);
            OBJ3 = getAreaByID(assignment[ka+1],arealist);
            OBJ2TASK = getPointsForTurning(OBJ1,OBJ2,OBJ3,OBJ1TASK);
            printstringvec(OBJ2TASK);
            obj2tasklen = OBJ2TASK.size();
            area_names.push_back(OBJ2TASK[0]);
            area_names.push_back(OBJ2TASK[1]);
            if (obj2tasklen == 6) {
                area_names.push_back(OBJ2TASK[2]);
                area_names.push_back(OBJ2TASK[3]);
                area_names.push_back(OBJ2TASK[4]);
                area_names.push_back(OBJ2TASK[5]);

                obj2wall_p0 = getPointByID(OBJ2TASK[0],pointlist);
                obj2wall_p1 = getPointByID(OBJ2TASK[1],pointlist);
                obj3wall_p0 = getPointByID(OBJ2TASK[2],pointlist);
                obj3wall_p1 = getPointByID(OBJ2TASK[3],pointlist);

                obj2frontwall_angle = atan2(obj2wall_p1.y-obj2wall_p0.y, obj2wall_p1.x-obj2wall_p0.x);
                obj3wall_angle = atan2(obj3wall_p1.y-obj3wall_p0.y, obj3wall_p1.x-obj3wall_p0.x);
                relative_angle = wrapToPi(obj3wall_angle-obj2frontwall_angle);

                if (OBJ2TASK[5].compare("right") == 0 && relative_angle < -SHARP_ANGLE_TRESHOLD) {
                    // Sharp angle to the right, we need to take the next wall into account as well
                    sharp_corner[ka+1] = true;
                } else if (OBJ2TASK[5].compare("left") == 0 && relative_angle > SHARP_ANGLE_TRESHOLD) {
                    // Sharp angle to the left, we need to take the next wall into account as well
                    sharp_corner[ka+1] = true;
                }
            }
            else
            {
                area_names.push_back("");
                area_names.push_back("");
                area_names.push_back("");
                area_names.push_back("");

            }
            OBJ_X_TASK.push_back(area_names);
        }

    }
}

void NapoleonAssignment::updateAreasAndFeatures(NapoleonPrediction &P)
{
    // Get old task counter
    P.u = P.pred_task_counter[P.j-1];
    if (P.j > 2) {
        uprev = P.pred_task_counter[P.j-2];
        if (P.u == uprev) {
            update_assignment = false;
        } else {
            update_assignment = true;
        }
    } else {
        update_assignment = true;
    }

    if (P.u < ka_max-1) {
        if (update_assignment) {
            area1ID = assignment[P.u];
            task1 = OBJ_X_TASK[P.u];
            area2ID = assignment[P.u+1];
            task2 = OBJ_X_TASK[P.u+1];
            if(P.u == (ka_max-2) )
            {
                area3ID = assignment[P.u+1];
                task3 = OBJ_X_TASK[P.u+1];
            }
            else
            {
                area3ID = assignment[P.u+2];
                task3 = OBJ_X_TASK[P.u+2];
            }

            curr_area = getAreaByID(area1ID,arealist);
            next_area = getAreaByID(area2ID,arealist);
            next_second_area = getAreaByID(area3ID,arealist);

        }

        // Bad but working statement to check if task2 contains 6 nonempty strings.
        // And if so, then it is considered as a corner assignment, otherwise as
        // go straight assignment.
        if (!task2[5].empty()) {
            if (update_assignment) {
                current_pivot = getPointByID(task2[4],pointlist);
                cur_next_hallway_rear = getPointByID(task2[2],pointlist);
                cur_next_hallway_front = getPointByID(task2[3],pointlist);
            }
            cur_pivot_local = coordGlobalToRopod(current_pivot, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
            cur_next_hallway_rear_local = coordGlobalToRopod(cur_next_hallway_rear, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
            cur_next_hallway_front_local = coordGlobalToRopod(cur_next_hallway_front, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
            cur_next_hallway_angle = atan2(cur_next_hallway_front_local.y-cur_next_hallway_rear_local.y,cur_next_hallway_front_local.x-cur_next_hallway_rear_local.x);
        } else if (!task2[1].empty()) { //sum(~cellfun(@isempty,task2),2) == 2
            if (update_assignment) {
                current_inter_rear_wall = getPointByID(task2[0],pointlist);
                current_inter_front_wall = getPointByID(task2[1],pointlist);
            }
            current_inter_rear_wall_local = coordGlobalToRopod(current_inter_rear_wall, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
            current_inter_front_wall_local = coordGlobalToRopod(current_inter_front_wall, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        }
    } else {
        // Final area from assignment
        area1ID = assignment[P.u];
        task1 = OBJ_X_TASK[P.u];
        P.pred_state[P.j] = 1;  // Cruising in last HW
    }

    curr_area = getAreaByID(area1ID,arealist);
    next_area = getAreaByID(area2ID,arealist);
    next_second_area = getAreaByID(area3ID,arealist);
}

void NapoleonAssignment::initializeAreas(NapoleonPrediction &P){
    P.u = P.pred_task_counter[0];
    if (P.u < ka_max-1)
    {
        area1ID = assignment[P.u];
        task1 = OBJ_X_TASK[P.u];
        area2ID = assignment[P.u+1];
        task2 = OBJ_X_TASK[P.u+1];
        if(P.u == (ka_max-2) )
        {
            area3ID = assignment[P.u+1];
            task3 = OBJ_X_TASK[P.u+1];
        }
        else
        {
            area3ID = assignment[P.u+2];
            task3 = OBJ_X_TASK[P.u+2];
        }
    }
}

void NapoleonAssignment::updateStateAndTask(NapoleonModel &M, NapoleonPrediction &P, NapoleonObstacle &O)
{
    // Taking a turn on an intersection
    if (!task2[5].empty()) {
        //printf("Ropod entry hall task2[5] non empty: %d\n",(int)pred_ropod_on_entry_hall[j]);
        if (P.pred_ropod_on_entry_hall[P.j] && P.pred_state[P.prevstate] == M.CRUSING) {
            P.pred_state[P.j] = M.CRUSING;
            //disp([num2str(i), ': Here we can switch to the next hallway']);
            P.u = P.u+1;
            area1ID = assignment[P.u];
            task1 = OBJ_X_TASK[P.u];
            if (P.u < ka_max-1) {
                area2ID = assignment[P.u+1];
                task2 = OBJ_X_TASK[P.u+1];
                if(P.u == (ka_max-2) )
                {
                    area3ID = assignment[P.u+1];
                    task3 = OBJ_X_TASK[P.u+1];
                }
                else
                {
                    area3ID = assignment[P.u+2];
                    task3 = OBJ_X_TASK[P.u+2];
                }
            }
            else
            {
                area1ID = assignment[ka_max-1];
                task1 = OBJ_X_TASK[ka_max-1];
                area2ID = assignment[ka_max-1];
                task2 = OBJ_X_TASK[ka_max-1];
                area3ID = assignment[ka_max-1];
                task3 = OBJ_X_TASK[ka_max-1];
            }
            curr_area = getAreaByID(area1ID,arealist);
            next_area = getAreaByID(area2ID,arealist);
            next_second_area = getAreaByID(area3ID,arealist);


        } else if (P.pred_ropod_on_entry_inter[P.j] && P.pred_state[P.prevstate] == M.CRUSING) {
            // If cruising and the y position of the ropod exceeds the y
            // position of the entry
            if(P.j==1)printf("Entry detected to turn intersection u = %d\n",P.u);
            P.pred_state[P.j] = M.ENTRY_BEFORE_TURN_ON_INTERSECTION;

        } else if (cur_pivot_local.x < SIZE_FRONT_ROPOD  && P.pred_state[P.prevstate] == M.ENTRY_BEFORE_TURN_ON_INTERSECTION) {
            // If in entry and the y position of the ropod exceeds the y
            // position of the intersection
            P.pred_state[P.j] = M.ACCELERATE_ON_INTERSECTION;

        } else if (cur_pivot_local.x <= SIZE_FRONT_RAX + START_STEERING_EARLY && P.pred_state[P.prevstate] == M.ACCELERATE_ON_INTERSECTION) {
            // If middle of vehicle is on y height of pivot
            P.pred_state[P.j] = M.ALIGN_AXIS_AT_INTERSECTION;

        } else if (cur_pivot_local.x <= -ROPOD_TO_AX+START_STEERING_EARLY && P.pred_state[P.prevstate] == M.ALIGN_AXIS_AT_INTERSECTION) {
            // If rearaxle is aligned with the pivot minus sse
            if(P.j==1)printf("Turning on  u = %d\n",P.u+1);
            P.pred_state[P.j] = M.TURNING;

        } else if (-ROTATED_ENOUGH_TRES < cur_next_hallway_angle && cur_next_hallway_angle < ROTATED_ENOUGH_TRES && P.pred_state[P.prevstate] == M.TURNING) {
            // If ropod has turned enough

            P.pred_state[P.j] = M.CRUSING;
            //disp([num2str(i), ': Here we can switch to the next task']);
            P.u = P.u+2;
            if(P.j==1)printf("Done with turning intersection, now on u = %d\n",P.u);
            area1ID = assignment[P.u];
            task1 = OBJ_X_TASK[P.u];
            if (P.u < ka_max-1)
            {
                area2ID = assignment[P.u+1];
                task2 = OBJ_X_TASK[P.u+1];
                if(P.u == (ka_max-2) )
                {
                    area3ID = assignment[P.u+1];
                    task3 = OBJ_X_TASK[P.u+1];
                }
                else
                {
                    area3ID = assignment[P.u+2];
                    task3 = OBJ_X_TASK[P.u+2];
                }
            }
            else
            {
                area1ID = assignment[ka_max-1];
                task1 = OBJ_X_TASK[ka_max-1];
                area2ID = assignment[ka_max-1];
                task2 = OBJ_X_TASK[ka_max-1];
                area3ID = assignment[ka_max-1];
                task3 = OBJ_X_TASK[ka_max-1];
            }
            curr_area = getAreaByID(area1ID,arealist);
            next_area = getAreaByID(area2ID,arealist);
            next_second_area = getAreaByID(area3ID,arealist);

        } else {

            P.pred_state[P.j] = P.pred_state[P.prevstate];
            P.update_state_points = false;
        }
        // Going straight on an intersection / ot between hallways
    } else if (!task2[1].empty()) {
        if (P.pred_ropod_on_entry_hall[P.j] && P.pred_state[P.prevstate] == M.CRUSING) {
            P.pred_state[P.j] = M.CRUSING;
            //disp([num2str(i), ': Here we can switch to the next hallway']);
            P.u = P.u+1;
            if(P.j==1)printf("Entry detected between halls u = %d\n",P.u);
            if (P.u < ka_max-1)
            {
                area1ID = assignment[P.u];
                task1 = OBJ_X_TASK[P.u];
                area2ID = assignment[P.u+1];
                task2 = OBJ_X_TASK[P.u+1];
                if(P.u == (ka_max-2) )
                {
                    area3ID = assignment[P.u+1];
                    task3 = OBJ_X_TASK[P.u+1];
                }
                else
                {
                    area3ID = assignment[P.u+2];
                    task3 = OBJ_X_TASK[P.u+2];
                }
            }
            else
            {
                area1ID = assignment[ka_max-1];
                task1 = OBJ_X_TASK[ka_max-1];
                area2ID = assignment[ka_max-1];
                task2 = OBJ_X_TASK[ka_max-1];
                area3ID = assignment[ka_max-1];
                task3 = OBJ_X_TASK[ka_max-1];
            }
            curr_area = getAreaByID(area1ID,arealist);
            next_area = getAreaByID(area2ID,arealist);
            next_second_area = getAreaByID(area3ID,arealist);


        }else if (P.pred_ropod_on_entry_inter[P.j] && P.pred_state[P.prevstate] == M.CRUSING) {
            // If cruising and the y position of the ropod exceeds the y
            // position of the entry
            if(P.j==1)printf("Entry detected to straight intersection u = %d\n",P.u);
            P.pred_state[P.j] = M.ENTRY_BEFORE_GOING_STRAIGHT_ON_INTERSECTION;
        } else if (current_inter_rear_wall_local.x < SIZE_FRONT_ROPOD+START_STEERING_EARLY && P.pred_state[P.prevstate] == M.ENTRY_BEFORE_GOING_STRAIGHT_ON_INTERSECTION) {
            // If in entry and the y position of the ropod exceeds the y
            // position of the intersection
            if(P.j==1)printf("Staright on u = %d\n",P.u+1);
            P.pred_state[P.j] = M.GOING_STRAIGHT_ON_INTERSECTION;
        } else if (current_inter_front_wall_local.x < -D_AX/2 && P.pred_state[P.prevstate] == M.GOING_STRAIGHT_ON_INTERSECTION) {

            P.pred_state[P.j] = M.CRUSING;
            P.u = P.u+2;
            if(P.j==1)printf("Done with straight intersection, now on u = %d\n",P.u);
            if (P.u < ka_max-1) {
                area1ID = assignment[P.u];
                task1 = OBJ_X_TASK[P.u];
                area2ID = assignment[P.u+1];
                task2 = OBJ_X_TASK[P.u+1];
                if(P.u == (ka_max-2) )
                {
                    area3ID = assignment[P.u+1];
                    task3 = OBJ_X_TASK[P.u+1];
                }
                else
                {
                    area3ID = assignment[P.u+2];
                    task3 = OBJ_X_TASK[P.u+2];
                }
            }
            else
            {
                area1ID = assignment[ka_max-1];
                task1 = OBJ_X_TASK[ka_max-1];
                area2ID = assignment[ka_max-1];
                task2 = OBJ_X_TASK[ka_max-1];
                area3ID = assignment[ka_max-1];
                task3 = OBJ_X_TASK[ka_max-1];
            }
            curr_area = getAreaByID(area1ID,arealist);
            next_area = getAreaByID(area2ID,arealist);
            next_second_area = getAreaByID(area3ID,arealist);


        } else {
            P.pred_state[P.j] = P.pred_state[P.prevstate];
            P.update_state_points = false;
        }
    }

    if(P.u >= ka_max)
    {
        P.u = ka_max-1;
    }

    if (P.pred_state[P.prevstate] == M.TIGHT_OVERTAKE || P.pred_state[P.prevstate] == M.SPACIOUS_OVERTAKE) {
        if (O.no_obs > 0) {
            O.current_obs_in_ropod_frame_pos = coordGlobalToRopod(O.obs_center_global, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
            //disp(['Obs is ',num2str(obs_in_ropod_frame_pos.x), ' m in front of ropod']);
            if (O.current_obs_in_ropod_frame_pos.x+O.current_obstacle.depth/2+D_AX+SIZE_REAR < 0) {
                P.pred_state[P.j] = M.CRUSING;
                P.update_state_points = true;
                P.pred_tube_width[P.j] = TUBE_WIDTH_C;
            } else {
                P.pred_state[P.j] = P.pred_state[P.prevstate];
                P.update_state_points = false;
            }
        } else {
            // Ropod doesn't see obs anymore, return cruising.
            // When obt disappears depends on the setting how long it takes before obs disappears after not seeing it (entity-timeout)
            // It can be found in /catkin_workspace/src/applications/ropod_navigation_test/config/model-example-ropod-navigation-ED.yaml
            P.pred_state[P.j] = M.CRUSING;
            P.update_state_points = true;
            P.pred_tube_width[P.j] = TUBE_WIDTH_C;
        }
    }

    // TODO: This code was moved. It was right before steerings were computed based on prediction. Check if still works on al cases
    // Exception for first plan
    if (P.j == 1) {
        P.update_state_points = true;
    }
    // Exception for when current and previous state are turning
    // And the state before that is aligning, as the state jump
    // happens immediately if that is required and then the state
    // points need to be updated.
    if (P.j > 1) {
        if (P.pred_state[P.j] == M.TURNING && P.pred_state[P.j-1] == M.TURNING && (P.pred_state[P.j-2] == M.ACCELERATE_ON_INTERSECTION || P.pred_state[P.j-2] == M.ALIGN_AXIS_AT_INTERSECTION)) {
            P.update_state_points = true;
        }
    }

    P.pred_task_counter[P.j] = P.u;

    // Monitor if we don't bump into front wall when aligning
    // rearaxle to pivot. If so, switch to turn state (5)
    if (P.pred_state[P.j] == M.ACCELERATE_ON_INTERSECTION || P.pred_state[P.j] == M.ALIGN_AXIS_AT_INTERSECTION) {
        if (P.update_state_points) {
            wall_front_p0 = getPointByID(task2[0],pointlist);
            wall_front_p1 = getPointByID(task2[1],pointlist);
            //global_wall_front_p0 = [wall_front_p0.x, wall_front_p0.y];
            //global_wall_front_p1 = [wall_front_p1.x, wall_front_p1.y];
        }

        local_wall_front_p0 = coordGlobalToRopod(wall_front_p0, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        local_wall_front_p1 = coordGlobalToRopod(wall_front_p1, P.pred_xy_ropod[P.j-1], P.pred_plan_theta[P.j-1]);
        if (do_lines_intersect(local_front_ropod_dilated_p0, local_front_ropod_dilated_p1, local_wall_front_p0, local_wall_front_p1)) {
            P.pred_state[P.j] = M.TURNING;
            P.update_state_points = true;
            //disp("Switched early while aligning pivot because too close to front wall");
        }
    }

}
