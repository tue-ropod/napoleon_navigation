//
// Created by bob on 25-09-19.
//

#ifndef NAVIGATION_BICYCLEMODEL_H
#define NAVIGATION_BICYCLEMODEL_H

#include "Model.h"

class BicycleModel : public Model{
protected:
    Pose2D steerWheel;
    Pose2D constrainedWheel;
    Vector2D velocity;
    int wheelOrder;

public:
    BicycleModel(Pose2D pose_, Polygon footprint_, double distanceSteerwheel, double distanceConstrainedwheel, double maxSpeed_ , double maxAcceleration_);
    void input(Vector2D velocity_, Frame frame = Frame_Robot);
    void update();
    void show(Visualization& canvas, Color c, int drawstyle);
    int follow(Tubes& tubes, Visualization& canvas);
};


#endif //NAVIGATION_BICYCLEMODEL_H
