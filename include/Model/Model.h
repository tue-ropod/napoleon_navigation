//
// Created by bob on 25-09-19.
//

#ifndef NAVIGATION_MODEL_H
#define NAVIGATION_MODEL_H

#include <utility>
#include <vector>
#include <Definitions/Polygon.h>
#include <Definitions/Pose2D.h>
#include <Visualization/Visualization.h>
#include <Obstacles/Obstacle.h>
#include <Tube/Tubes.h>

enum Frame {Frame_Robot, Frame_World};
enum FollowStatus {Status_Ok, Status_Collision, Status_Stuck, Status_Error, Status_Done};

class Model {
public:
    Polygon footprint;
    Polygon dilatedFootprint;
    double footprintClearance = 0.03;
    Pose2D pose;
    double maxSpeed;
    double maxAcceleration;
    double maxRotationalSpeed;
    double maxRotationalAcceleration;
    Circle scanradius;
    double footprintMultiplier = 1.1;
    double footprintscalex = 1, footprintscaley = 1;

    Model(Pose2D pose_, Polygon footprint_, double maxSpeed_, double maxAcceleration_, double wheelDistanceToMiddle_);
    virtual void show(Visualization& canvas, Color c = Color(255,0,0), int drawstyle = Filled);
    bool collision(Obstacle& o);
    void scaleFootprint(double x, double y);
    void dilateFootprint(double offset);
};

#endif //NAVIGATION_MODEL_H
