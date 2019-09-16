#ifndef NAP_VIS_H
#define NAP_VIS_H

#include <vector>
#include "napoleon_geometry.h"
#include <ros/ros.h>
#include <ropod_ros_msgs/GoToAction.h>
#include <ropod_ros_msgs/RoutePlannerAction.h>
#include <visualization_msgs/Marker.h>
#include "napoleon_assignment.h"

using namespace std;

class NapoleonVisualization {

public:

    visualization_msgs::Marker vis_points;
    visualization_msgs::Marker vis_wall;
    visualization_msgs::Marker vis_plan;
    Point vis_rt, vis_lt, vis_fr, vis_fl;

public:

    NapoleonVisualization()
    {
    }

    void visualizeRopodMarkers(ros::Publisher &ropodmarker_pub, NapoleonPrediction &P);
    void showWallPoints(Point local_wallpoint_front, Point local_wallpoint_rear,  ros::Publisher &pub);
    void initializeVisualizationMarkers(NapoleonAssignment &A);
    void visualizePlan(ros::Publisher &mapmarker_pub, NapoleonAssignment &A, NapoleonPrediction &P);
    void publish(ros::Publisher &ropodmarker_pub, NapoleonModel &M, NapoleonPrediction &P, NapoleonAssignment &A);

};

#endif
