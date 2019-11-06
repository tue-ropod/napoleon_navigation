//
// Created by bob on 25-09-19.
//

#include "BicycleModel.h"

BicycleModel::BicycleModel(Pose2D pose_, Polygon footprint_, double distanceSteerwheel, double distanceConstrainedwheel, double maxSpeed_, double maxAcceleration_) :
    Model(pose_, std::move(footprint_), maxSpeed_, maxAcceleration_, abs(distanceSteerwheel - distanceConstrainedwheel)){
    steerWheel = pose;
    steerWheel.transformThis(distanceSteerwheel * cos(pose.a), distanceSteerwheel * sin(pose.a), 0);
    constrainedWheel = pose;
    constrainedWheel.transformThis(distanceConstrainedwheel * cos(pose.a), distanceConstrainedwheel * sin(pose.a), 0);
    velocity = Vector2D(0,0);
    wheelOrder = distanceSteerwheel > distanceConstrainedwheel ? 1 : -1;
}

void BicycleModel::input(Vector2D velocity_, Frame frame) {
    if(frame == Frame_World){
        velocity_.transformThis(0, 0, -pose.a + M_PI_2);
    }
    velocity = velocity_;
    velocity.transformThis(0, 0, -M_PI_2); //rotate object frame in world frame
}

void BicycleModel::update() {

    double delta = velocity.angle();
    double vx = velocity.length()*cos(delta)*cos(constrainedWheel.a);
    double vy = velocity.length()*cos(delta)*sin(constrainedWheel.a);
    double vtheta = velocity.length() * (1/(steerWheel.distance(constrainedWheel))) * sin(delta) * wheelOrder;

    pose.transformThis(-constrainedWheel.x, -constrainedWheel.y, 0);
    steerWheel.transformThis(-constrainedWheel.x, -constrainedWheel.y, 0);

    constrainedWheel.rotateOrientation(vtheta);
    constrainedWheel.transformThis(vx, vy, 0);

    pose.transformThis(0, 0, vtheta);
    pose.rotateOrientation(vtheta);

    steerWheel.transformThis(0, 0, vtheta);
    steerWheel.a = delta + pose.a;

    pose.transformThis(constrainedWheel.x, constrainedWheel.y, 0);
    steerWheel.transformThis(constrainedWheel.x, constrainedWheel.y, 0);

    footprint.transformto(pose);
}

void BicycleModel::show(Visualization& canvas, Color c, int drawstyle) {
    canvas.polygon(footprint.vertices, Color(255,0,0), Thin);

    canvas.point(constrainedWheel, c, Thick);

    Vector2D dir(cos(steerWheel.a), sin(steerWheel.a));
    dir.scaleThis(5, 5);
    dir.transformThis(steerWheel.x, steerWheel.y, 0);
    canvas.arrow(steerWheel, dir, c, Thin);
}

int BicycleModel::follow(Tubes& tubes, Visualization& canvas){

    Tube tube;
    bool inTube = false;
    int tubeIndex = -1;

    int i = 0;
    for(auto & t : tubes.tubes){
        if( t.shape.polygonContainsPoint(pose) ){
            tube = t;
            inTube = true;
            tubeIndex = i;
        }
        i ++;
    }

    if( inTube ) {
        double speed = maxSpeed * tube.velocity.length();

        double tubeDirection = tube.velocity.angle();
        double angleDiff = tubeDirection - pose.a;

        angleDiff = fmod(angleDiff + M_PI, M_PI*2);
        if (angleDiff < 0) angleDiff += M_PI*2;
        angleDiff -= M_PI;

        double alignFactor = cos(angleDiff);
        alignFactor = alignFactor < 0 ? 0 : alignFactor;

        Vector2D v = tube.velocity;

        vector<Line> sides = tube.shape.sidesPolygonPolygonCollision(footprint);
        if(!sides.empty()){
            for(auto & side : sides){
                double dist = side.lineDistanceToPoint(pose);
                dist = dist < 0.01 ? 0.01 : dist;
                Vector2D projectionPoint = side.lineProjectionPoint(pose);
                canvas.point(projectionPoint, Color(0,255,0),Thick);
                Vector2D steerVector = ((pose.toVector() - projectionPoint).unit() * (1 / dist));
                v = v + steerVector;
            }
        }

        v = v.unit()*speed;

        input(Vector2D(v.x, v.y), Frame_World);
    } else{
        input(Vector2D(0,0));
    }

    return tubeIndex;
}