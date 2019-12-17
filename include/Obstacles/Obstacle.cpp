//
// Created by bob on 01-10-19.
//

#include "Obstacle.h"

Obstacle::Obstacle(Polygon footprint_, Pose2D pose_, Physics physics, int weight_){
    footprint = std::move(footprint_);
    pose = pose_;
    footprint.transformto(pose);
    movement = Pose2D();
    dynamic = bool(physics);
    lifeTime = 0.0;
    weight = weight_;
}

void Obstacle::show(Visualization& canvas, Color c, int drawstyle) {
    if(lifeTime == -1){c = Color(100,100,100);}
    else{
        int a = lifeTime < 10 && lifeTime >= 0 ? int(lifeTime) : 10;
        c.alpha = c.alpha - a*20;
    }
    canvas.polygon(footprint.vertices, c, drawstyle);
}

void Obstacle::update(double dt) {
    if (dynamic) {
        pose = pose + movement*dt;
        footprint.transformto(pose);
    }
    if(lifeTime >= 0) {
        lifeTime += dt;
    }
}