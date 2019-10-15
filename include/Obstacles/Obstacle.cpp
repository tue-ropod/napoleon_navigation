//
// Created by bob on 01-10-19.
//

#include "Obstacle.h"

Obstacle::Obstacle(Polygon footprint_, Pose2D pose_, Physics physics){
    footprint = std::move(footprint_);
    pose = pose_;
    footprint.transformto(pose);
    movement = Pose2D();
    dynamic = bool(physics);
}

void Obstacle::show(Visualization& canvas, Color c, int drawstyle) {
    canvas.polygon(footprint.vertices, c, drawstyle);
}

void Obstacle::update() {
    if (dynamic) {
        pose = pose + movement;
        footprint.transformto(pose);
    }
}