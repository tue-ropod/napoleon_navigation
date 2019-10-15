//
// Created by bob on 25-09-19.
//

#include "Model.h"

Model::Model(Pose2D pose_, Polygon footprint_, double maxSpeed_, double maxAcceleration_, double wheelDistanceToMiddle_){
    pose = pose_;
    footprint = std::move(footprint_);
    dilatedFootprint = footprint;
    footprint.transformto(pose);
    dilatedFootprint = footprint;
    scanradius = Circle(Vector2D(pose.x, pose.y), 30);
    maxSpeed = maxSpeed_;
    maxAcceleration = maxAcceleration_;
    maxRotationalSpeed = maxSpeed / wheelDistanceToMiddle_;
    maxRotationalAcceleration = maxAcceleration / wheelDistanceToMiddle_;
}

bool Model::collision(Obstacle& o){
    //sequence only check sides collisions of a polygon if the middle is not inside.
    return (footprint.polygonContainsPoint(o.footprint.middle) || footprint.polygonPolygonCollision(o.footprint));
}

void Model::scaleFootprint(double x, double y) {
    footprint.transformto(Pose2D(0,0,0));
    footprint.scale(1/footprintscalex, 1/footprintscaley);
    footprint.scale(x, y);
    footprint.transformto(pose);
    footprintscalex = x;
    footprintscaley = y;
}

void Model::dilateFootprint(double offset){
    dilatedFootprint.dilate(offset);
}


void Model::show(Visualization& canvas, Color c, int drawstyle) {
    canvas.polygon(footprint.vertices, Color(255,0,0), Thin);
    double sx = footprintscalex, sy = footprintscaley;
    scaleFootprint(1,1);
    canvas.polygon(footprint.vertices, c, drawstyle);
    scaleFootprint(sx,sy);
}