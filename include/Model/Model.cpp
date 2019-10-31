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
    currentTubeIndex = 0;
    speedScale = 1;
}

bool Model::collision(Obstacle& o){
    //sequence only check sides collisions of a polygon if the middle is not inside.
    return (footprint.polygonContainsPoint(o.footprint.middle) || footprint.polygonPolygonCollision(o.footprint));
}

double Model::minWidth(){
    vector<Vector2D> vertex = dilatedFootprint.boundingBoxRotated(pose.a);
    double l1 = vertex[0].distance(vertex[1]);
    double l2 = vertex[1].distance(vertex[2]);
    return l1 < l2 ? l1 : l2;
}

double Model::maxWidth(){
    vector<Vector2D> vertex = dilatedFootprint.boundingBoxRotated(pose.a);
    double l1 = vertex[0].distance(vertex[1]);
    double l2 = vertex[1].distance(vertex[2]);
    return sqrt(l1*l1 + l2*l2);
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

void Model::changeSpeedScale(double x) {
    speedScale += x;
    speedScale = speedScale > 1 ? 1 : speedScale;
    speedScale = speedScale < 0 ? 0 : speedScale;
}

void Model::copySettings(Model &modelCopy) {
    speedScale = modelCopy.speedScale;
    predictionBiasVelocity = modelCopy.predictionBiasVelocity;
}

void Model::copyState(Model &modelCopy) {
    predictionBiasVelocity = modelCopy.predictionBiasVelocity;
    velocity = modelCopy.velocity;
    inputVelocity = modelCopy.inputVelocity;
    desiredVelocity = modelCopy.desiredVelocity;
    pose = modelCopy.pose;
    footprint.transformto(pose);
    dilatedFootprint.transformto(pose);
}

void Model::calculateInputVelocity(double dt){


    Pose2D acc = (desiredVelocity - velocity)/dt;
    if(acc.length() > maxAcceleration){
        Vector2D scaledAcc = acc.unit()*maxAcceleration;
        acc.x = scaledAcc.x;
        acc.y = scaledAcc.y;
    }

    if(abs(acc.a) > maxRotationalAcceleration){
        acc.a = (acc.a/abs(acc.a))*maxRotationalAcceleration;
    }

    inputVelocity = velocity + acc * dt;

    if(inputVelocity.length() > maxSpeed){
        Vector2D scaledVel = inputVelocity.unit()*maxSpeed;
        inputVelocity.x = scaledVel.x;
        inputVelocity.y = scaledVel.y;
    }
    if(abs(inputVelocity.a) > maxRotationalSpeed){
        inputVelocity.a = (inputVelocity.a/abs(inputVelocity.a))*maxRotationalSpeed;
    }
}

void Model::update(double dt, Communication &comm) {
    if(comm.newPosition()){
        pose = comm.measuredPose;
    }
    if(comm.newOdometry()){
        Pose2D measuredVelocity = comm.measuredVelocity;
        measuredVelocity.transformThis(0,0,pose.a);
        velocity = velocity*0.7 + Pose2D(measuredVelocity.x, measuredVelocity.y, measuredVelocity.a)*0.3;
    }

    if(applyBrake){
        inputVelocity = Pose2D(0,0,0);
    }else {
        calculateInputVelocity(dt);
    }
    velocity = inputVelocity;
    updateModel(dt);

    Pose2D vel = inputVelocity;
    vel.transformThis(0, 0, -pose.a);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vel.x;
    cmd_vel.linear.y = vel.y;
    cmd_vel.angular.z = vel.a;
    comm.setVel(cmd_vel);

    applyBrake = false;
}

void Model::brake(){
    applyBrake = true;
}

FollowStatus Model::predict(int nScalings, double predictionTime, double minPredictionDistance, double dt, Model &origionalModel, Tubes &tubes, Visualization &canvas) {
    FollowStatus status = Status_Error;

    changeSpeedScale(1/double(nScalings));
    //changeSpeedScale(1);

    for(int s = 0; s < nScalings; s++) {
        copyState(origionalModel);
        Vector2D prevPos = pose.toVector();
        double distance = 0;
        double N = ceil(predictionTime / dt);
        double percentage = 0.5;
        double beginPercentage = 1;
        Pose2D finalPredictionBias = predictionBiasVelocity * percentage;
        for (int p = 0; p < N; p++) {
            status = follow(tubes, canvas, false);
            updatePrediction(dt);
            distance += prevPos.distance(pose.toVector());
            prevPos = pose.toVector();
            beginPercentage = beginPercentage - percentage;
            percentage = beginPercentage * percentage;
            finalPredictionBias = finalPredictionBias + predictionBiasVelocity * percentage;
            //show(canvas, Color(255,255,255), Thin);
            if (status != Status_Ok && status != Status_Collision) { break; }
        }
        predictionBiasVelocity = finalPredictionBias;
        show(canvas, Color(255, 255, 255), Thin);
        if (distance < minPredictionDistance && status != Status_Done) {
            status = Status_ToClose;
        }
        if (status != Status_Ok && status != Status_Done) {
            changeSpeedScale(-1/double(nScalings));
            if (speedScale == 0) { break; }
        } else { break; }
    }

    //show(canvas, Color(255, 255, 255), Thin);

    return status;
}

void Model::showCommunicationInput(Visualization& canvas, Color c, int drawstyle, Communication &comm) {
    Pose2D mPose = comm.measuredPose;
    Vector2D posdir = Vector2D(0.5,0).transform(0,0,mPose.a);
    Vector2D pos = mPose.toVector();
    canvas.arrow(pos, pos+posdir, c, drawstyle);

    Vector2D offset = Vector2D(1,0).transform(0,0,mPose.a) + pos;
    Vector2D dir = Vector2D(0,1).transform(0,0,mPose.a)*comm.measuredVelocity.a;
    canvas.arrow(offset, offset+dir, Color(0,0,255), drawstyle);

    canvas.arrow(pos, pos+comm.measuredVelocity.toVector().transform(0,0,mPose.a), Color(0,0,255), drawstyle);
}


////////////////////////////////////Virtual functions///////////////////////////////////////////////////////////////////

void Model::show(Visualization& canvas, Color c, int drawstyle) {}

FollowStatus Model::follow(Tubes &tubes, Visualization &canvas, bool debug) {
    return Status_Error;
}

void Model::updateModel(double dt){}

void Model::updatePrediction(double dt) {}

void Model::input(Pose2D velocity_, Frame frame){}

Pose2D Model::translateInput(Vector2D position, Pose2D velocity_){
    return {};
}

