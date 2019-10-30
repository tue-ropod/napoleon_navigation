//
// Created by bob on 25-09-19.
//

#ifndef NAVIGATION_HOLONOMICMODEL_H
#define NAVIGATION_HOLONOMICMODEL_H

#include "Model.h"

class HolonomicModel : public Model {

public:
    HolonomicModel(Pose2D pose_, Polygon footprint_, double maxSpeed_, double maxAcceleration_, double wheelDistanceToMiddle_);

    void input(Pose2D velocity_, Frame frame) override ;
    Pose2D translateInput(Vector2D position, Pose2D velocity_) override;
    void updateModel(double dt) override;
    void updatePrediction(double dt) override;
    void show(Visualization& canvas, Color c, int drawstyle) override;
    FollowStatus follow(Tubes& tubes, Visualization& canvas, bool debug) override;
};

#endif //NAVIGATION_HOLONOMICMODEL_H
