#ifndef NAP_ASS_H
#define NAP_ASS_H

#include <ros/ros.h>
#include <vector>
#include "napoleon_geometry.h"
#include "napoleon_config.h"
#include "napoleon_functions.h"
#include "napoleon_prediction.h"
#include <ropod_ros_msgs/GoToAction.h>
#include <ropod_ros_msgs/RoutePlannerAction.h>

class NapoleonAssignment{

public:

    std::vector<PointID> pointlist;
    std::vector<AreaQuadID> arealist;
    std::vector<int> assignment;

    AreaQuadID cur_obj;
    vector<string> areaIDs;
    int ka_max;  // Assignment length
    int uprev;        

    AreaQuadID current_hallway;
    AreaQuadID next_hallway;
    AreaQuadID curr_area;
    AreaQuadID next_area;
    AreaQuadID next_second_area;

    std::vector<std::vector<string>> OBJ_X_TASK;
    std::vector<std::string> task1, task2, task3;
    std::vector<std::string> current_hallway_task, next_hallway_task;

    int area1ID, area2ID, area3ID;

    PointID current_pivot, cur_next_hallway_rear, cur_next_hallway_front;
    PointID current_inter_rear_wall, current_inter_front_wall;
    Point cur_pivot_local, cur_next_hallway_rear_local, cur_next_hallway_front_local;
    Point current_inter_rear_wall_local, current_inter_front_wall_local;
    double cur_next_hallway_angle;
    PointID wall_front_p0, wall_front_p1;
    Point local_wall_front_p0, local_wall_front_p1;
    Point local_front_ropod_dilated_p0 = Point(DILATE_ROPOD_ALIGNING, -DILATE_ROPOD_ALIGNING);
    Point local_front_ropod_dilated_p1 = Point(DILATE_ROPOD_ALIGNING, DILATE_ROPOD_ALIGNING);
    PointID point_rear, point_front;
    Point glob_wallpoint_front, glob_wallpoint_rear;
    Point global_wall_front_p0, global_wall_front_p1;
    PointID point_pivot;
    Point local_wallpoint_front, local_wallpoint_rear;
    Point local_pivot;

    PointID obj2wall_p0, obj2wall_p1, obj3wall_p0, obj3wall_p1;
    double obj2frontwall_angle, obj3wall_angle, relative_angle;

    bool update_assignment;

    bool sharp_corner[100] = {false}; // quick but dirty fix! - now size is fixed to 100 since we don't
                                      // have number of areas value during compilation time
                                      // This will fail if number of areas go above 100

    std::vector<ropod_ros_msgs::Area> planner_areas;

public:

    NapoleonAssignment(){
    }

    void initializeAssignment();
    void updateAreasAndFeatures(NapoleonPrediction &P);
    void initializeAreas(NapoleonPrediction &P);
    void updateStateAndTask();

};

#endif
