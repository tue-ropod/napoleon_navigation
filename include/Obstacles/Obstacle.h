//
// Created by bob on 01-10-19.
//

#ifndef NAVIGATION_OBSTACLE_H
#define NAVIGATION_OBSTACLE_H


#include <Definitions/Polygon.h>
#include <utility>

enum Physics {Static = 0, Dynamic = 1};

class Obstacle {
protected:
public:
    Polygon footprint;
    Pose2D pose;
    Pose2D movement;
    bool dynamic;
    double lifeTime;

    Obstacle(Polygon footprint_, Pose2D pose_ = Pose2D(), Physics physics = Static);
    void show(Visualization& canvas, Color c, int drawstyle = Thin);
    void update(double dt);
};


#endif //NAVIGATION_OBSTACLE_H
