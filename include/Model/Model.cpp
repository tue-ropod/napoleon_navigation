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
    maxSpeed = maxSpeed_;
    maxAcceleration = maxAcceleration_;
    maxRotationalSpeed = maxSpeed / wheelDistanceToMiddle_;
    maxRotationalAcceleration = maxAcceleration / wheelDistanceToMiddle_;
}

bool Model::checkCollision(Obstacles& obstacles){
    bool collision = false;
    vector<Vector2D> boundingBox = dilatedFootprint.boundingBoxRotated(pose.a);
    Polygon left = Polygon({boundingBox[0], boundingBox[1], pose.toVector()},Closed);
    Polygon right = Polygon({boundingBox[2], boundingBox[3], pose.toVector()},Closed);
    Polygon front = Polygon({boundingBox[1], boundingBox[2], pose.toVector()},Closed);
    Polygon back = Polygon({boundingBox[0], boundingBox[3], pose.toVector()},Closed);
    bool leftcheck = false, rightcheck = false, frontcheck = false, backcheck = false;
    limitedMovements.clear();
    for(auto &obstacle : obstacles.obstacles){
        //sequence only check sides collisions of a polygon if the middle is not inside.
        if((dilatedFootprint.polygonContainsPoint(obstacle.footprint.middle) || dilatedFootprint.polygonPolygonCollision(obstacle.footprint))){
            //check which side of the bounding box has collision and then add that side to the limited movements
            if(!leftcheck && obstacle.footprint.polygonPolygonCollision(left)){limitedMovements.emplace_back(Movement_Left); leftcheck = true;}
            else if(!rightcheck && obstacle.footprint.polygonPolygonCollision(right)){limitedMovements.emplace_back(Movement_Right); rightcheck = true;}
            else if(!frontcheck && obstacle.footprint.polygonPolygonCollision(front)){limitedMovements.emplace_back(Movement_Forward); frontcheck = true;}
            else if(!backcheck && obstacle.footprint.polygonPolygonCollision(back)){limitedMovements.emplace_back(Movement_Backward); backcheck = true;}
            else {break;}
            status = Status_ObstacleCollision;
            collision = true;
        }
    }
    return collision;
}

double Model::width(){
    vector<Vector2D> vertex = dilatedFootprint.boundingBoxRotated(pose.a);
    return vertex[1].distance(vertex[2]);
}

double Model::length(){
    vector<Vector2D> vertex = dilatedFootprint.boundingBoxRotated(pose.a);
    return vertex[0].distance(vertex[1]);
}

double Model::turnWidth(){
    vector<Vector2D> vertex = dilatedFootprint.boundingBoxRotated(pose.a);
    double l1 = vertex[0].distance(vertex[1])/2;
    double l2 = vertex[1].distance(vertex[2]);
    return sqrt(l1*l1 + l2*l2);
}

double Model::brakeDistance(){
    double v = velocity.length();
    double a = maxAcceleration;
    return 0.5*(v*v)/maxAcceleration;
}

void Model::dilateFootprint(double offset){
    dilatedFootprint.dilate(offset);
}

void Model::changeSpeedScale(double x) {
    speedScale = x;
    speedScale = speedScale > 1 ? 1 : speedScale;
    speedScale = speedScale < 0 ? 0 : speedScale;
}

void Model::copySettings(Model &modelCopy) {
    speedScale = modelCopy.speedScale;
    predictionBiasVelocity = modelCopy.predictionBiasVelocity;
}

void Model::copyState(Model &modelCopy) {
    velocity = modelCopy.velocity;
    inputVelocity = modelCopy.inputVelocity;
    desiredVelocity = modelCopy.desiredVelocity;
    pose = modelCopy.pose;
    footprint.transformto(pose);
    dilatedFootprint.transformto(pose);
}

void Model::update(double dt, Communication &comm) {
    if(!poseInitialized){
        poseInitialized = true;
        pose = comm.measuredPose;
        Pose2D measuredVelocity = comm.measuredVelocity;
        measuredVelocity.transformThis(0,0,pose.a);
        velocity = Pose2D(measuredVelocity.x, measuredVelocity.y, measuredVelocity.a);
    }else{
        pose = comm.measuredPose;
        Pose2D measuredVelocity = comm.measuredVelocity;
        measuredVelocity.transformThis(0,0,pose.a);
        velocity = velocity * 0.9 + Pose2D(measuredVelocity.x, measuredVelocity.y, measuredVelocity.a) * 0.1;
    }

    updatePrediction(dt);

    Pose2D vel = inputVelocity;
    vel.transformThis(0, 0, -pose.a);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = vel.x;
    cmd_vel.linear.y = vel.y;
    cmd_vel.angular.z = vel.a;
    comm.setVel(cmd_vel);
}

void Model::brake(){
    applyBrake = true;
}

FollowStatus Model::predict(double dt, Model &origionalModel, Tubes &tubes, Communication &comm, Visualization &canvas) {
    status = Status_Error;
    double brakeMargin = 1;
    double minSpeedScale = 0.2;
    changeSpeedScale(speedScale*1.01);
    //changeSpeedScale(1);

    for(int s = 0; s < comm.nTries_param; s++) {
        copyState(origionalModel);
        Vector2D prevPos = pose.toVector();
        double distance = 0;
        double N = ceil(comm.predictionTime_param / dt);
        Pose2D finalPredictionBias;
        int nLengthsAhead = 0;
        for (int p = 0; p < N; p++) {
            status = follow(tubes, comm, canvas, false);
            updatePrediction(dt);
            distance += prevPos.distance(pose.toVector());
            prevPos = pose.toVector();
            finalPredictionBias = finalPredictionBias + predictionBiasVelocity * ((N-p)/N);
            if(int(floor(distance / origionalModel.length())) == nLengthsAhead && distance <= brakeDistance()+brakeMargin+origionalModel.length()){
                show(canvas, Color(255,255,255), Thin);
                checkCollision(comm.obstacles);
                nLengthsAhead++;
            }
            if (status != Status_Ok) { break; }
        }
        finalPredictionBias.constrainThis(velocity.toVector().length(), velocity.a);
        predictionBiasVelocity = finalPredictionBias;
        show(canvas, Color(255, 255, 255), Thin);
        if (distance < comm.minPredictionDistance_param && status == Status_Ok) {
            status = Status_ShortPredictionDistance;
        }
        if ((status == Status_TubeCollision || status == Status_Stuck) && speedScale > minSpeedScale) {
            changeSpeedScale(speedScale*0.8);
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

void Model::showStatus(const string& modelName) {
    if(status != prevStatus) {
        switch (status) {
            case Status_Ok: {cout << modelName + " status: Ok" << endl;break;}
            case Status_ShortPredictionDistance: {cout << modelName + " status: short prediction distance" << endl;break;}
            case Status_Stuck: {cout << modelName + " status: stuck" << endl;break;}
            case Status_Error: {cout << modelName + " status: error" << endl;break;}
            case Status_ObstacleCollision: {cout << modelName + " status: obstacle collision" << endl;break;}
            case Status_TubeCollision: {cout << modelName + " status: tube collision" << endl;break;}
            case Status_OutsideTube: {cout << modelName + " status: outside tube" << endl;break;}
            case Status_Recovering: {cout << modelName + " status: recovering" << endl;break;}
            case Status_WrongWay: {cout << modelName + " status: wrong way" << endl;break;}
            case Status_Done: {cout << modelName + " status: done" << endl;break;}
        }
        prevStatus = status;
    }
}

////////////////////////////////////Virtual functions///////////////////////////////////////////////////////////////////

void Model::show(Visualization& canvas, Color c, int drawstyle) {}

FollowStatus Model::follow(Tubes &tubes, Communication& comm, Visualization &canvas, bool debug) {
    return Status_Error;
}

void Model::updateModel(double dt){}

void Model::updatePrediction(double dt) {}

void Model::input(Pose2D velocity_, Frame frame){}

Pose2D Model::translateInput(Vector2D position, Pose2D velocity_){
    return {};
}

