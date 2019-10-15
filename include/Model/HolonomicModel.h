//
// Created by bob on 25-09-19.
//

#ifndef NAVIGATION_HOLONOMICMODEL_H
#define NAVIGATION_HOLONOMICMODEL_H

#include "Model.h"

class HolonomicModel : public Model {
protected:
    Pose2D velocity, inputVelocity;

public:
    HolonomicModel(Pose2D pose_, Polygon footprint_, double maxSpeed_, double maxAcceleration_, double wheelDistanceToMiddle_);
    void input(Pose2D velocity_, Frame frame = Frame_Robot);
    Pose2D translateInput(Vector2D position, Pose2D velocity_);
    void update(double dt);
    void show(Visualization& canvas, Color c, int drawstyle);
    FollowStatus follow(Tubes& tubes, int &tubeIndex, double speedScale, Visualization& canvas, bool debug = false);
};

#endif //NAVIGATION_HOLONOMICMODEL_H
