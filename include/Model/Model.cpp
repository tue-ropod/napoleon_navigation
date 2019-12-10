//
// Created by bob on 25-09-19.
//

#include "Model.h"

Model::Model(Pose2D pose_, Polygon footprint_, double maxSpeed_, double maxAcceleration_, double wheelDistanceToMiddle_){
    pose = pose_;
    footprint = std::move(footprint_);
    footprint.transformto(pose);
    dilatedFootprint = footprint;
    maxSpeed = maxSpeed_;
    maxAcceleration = maxAcceleration_;
    maxRotationalSpeed = maxSpeed / wheelDistanceToMiddle_;
    maxRotationalAcceleration = maxAcceleration / wheelDistanceToMiddle_;

    vector<Vector2D> scanAreaVertices;
    scanAreaVertices.emplace_back(Vector2D(0,0));
    double r = 3;
    double angle = 200*(2*M_PI/360);
    double N = 20;
    for(int n = 0; n < N; n++){
        double a = (angle/(N-1))*n;
        scanAreaVertices.emplace_back(Vector2D(r*cos(a), r*sin(a)));
    }
    scanArea = Polygon(scanAreaVertices, Closed, true, Pose2D(0,0,M_PI-(2*M_PI-angle)/2));
    scanArea.transformto(pose);
}

bool Model::checkCollision(Obstacles& obstacles, Visualization &canvas){
    bool collision = false;
    Polygon obstacleCollisionFootprint;
    obstacleCollisionFootprint = dilatedFootprint;
    double Xplus = cos(velocity.angle()-pose.a)*brakeDistance();
    double Xmin = -cos(velocity.angle()-pose.a)*brakeDistance();
    double Yplus = sin(velocity.angle()-pose.a)*brakeDistance();
    double Ymin = -sin(velocity.angle()-pose.a)*brakeDistance();
    Xplus = Xplus < 0 ? 0 : Xplus;
    Xmin = Xmin < 0 ? 0 : Xmin;
    Yplus = Yplus < 0 ? 0 : Yplus;
    Ymin = Ymin < 0 ? 0 : Ymin;
    obstacleCollisionFootprint.dilateDirection(Xplus,Xmin,Yplus,Ymin,pose.a);

    vector<Vector2D> boundingBox = obstacleCollisionFootprint.boundingBoxRotated(pose.a);
    Polygon left = Polygon({boundingBox[0], boundingBox[1], pose.toVector()}, Closed);
    Polygon right = Polygon({boundingBox[2], boundingBox[3], pose.toVector()}, Closed);
    Polygon front = Polygon({boundingBox[1], boundingBox[2], pose.toVector()}, Closed);
    Polygon back = Polygon({boundingBox[0], boundingBox[3], pose.toVector()}, Closed);
    Vector2D leftSideMiddle = (boundingBox[2]+boundingBox[3])/2;
    Vector2D rightSideMiddle = (boundingBox[0]+boundingBox[1])/2;
    Polygon rotateLeft = Polygon({boundingBox[0], rightSideMiddle, pose.toVector(), leftSideMiddle, boundingBox[2], pose.toVector()}, Closed);
    Polygon rotateRight = Polygon({boundingBox[1], rightSideMiddle, pose.toVector(), leftSideMiddle, boundingBox[3], pose.toVector()}, Closed);
    bool leftCheck = false, rightCheck = false, frontCheck = false, backCheck = false, rotateLeftCheck = false, rotateRightCheck = false;
    for(auto &obstacle : obstacles.obstacles){
        if( (obstacleCollisionFootprint.polygonPolygonCollision(obstacle.footprint)) ){
            //check which side of the bounding box has collision and then add that side to the limited movements
            if(!leftCheck && obstacle.footprint.polygonPolygonCollision(left)){limitedMovements.emplace_back(Movement_Left); leftCheck = true;}
            if(!rightCheck && obstacle.footprint.polygonPolygonCollision(right)){limitedMovements.emplace_back(Movement_Right); rightCheck = true;}
            if(!frontCheck && obstacle.footprint.polygonPolygonCollision(front)){limitedMovements.emplace_back(Movement_Forward); frontCheck = true;}
            if(!backCheck && obstacle.footprint.polygonPolygonCollision(back)){limitedMovements.emplace_back(Movement_Backward); backCheck = true;}
            if(!rotateLeftCheck && obstacle.footprint.polygonPolygonCollision(rotateLeft)){limitedMovements.emplace_back(Movement_RotateLeft); rotateLeftCheck = true;}
            if(!rotateRightCheck && obstacle.footprint.polygonPolygonCollision(rotateRight)){limitedMovements.emplace_back(Movement_RotateRight); rotateRightCheck = true;}
            if(leftCheck && rightCheck && frontCheck && backCheck && rotateLeftCheck && rotateRightCheck){break;}
            status = Status_ObstacleCollision;
            collision = true;
        }
    }
    canvas.idName = "Collision_prevention";
    canvas.polygon(boundingBox, Color(0,0,200), Thin);
    if(leftCheck){canvas.polygon(left.vertices, Color(0,0,255), Thick);}
    if(rightCheck){canvas.polygon(right.vertices, Color(0,0,255), Thick);}
    if(frontCheck){canvas.polygon(front.vertices, Color(0,0,255), Thick);}
    if(backCheck){canvas.polygon(back.vertices, Color(0,0,255), Thick);}
    if(rotateLeftCheck){canvas.polygon(rotateLeft.vertices, Color(0,0,255), Thick);}
    if(rotateRightCheck){canvas.polygon(rotateRight.vertices, Color(0,0,255), Thick);}
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
    return 0.5*(v*v)/a;
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
    for(Movement &movement : modelCopy.limitedMovements){
        limitedMovements.emplace_back(movement);
    }
}

void Model::copyState(Model &modelCopy) {
    velocity = modelCopy.velocity;
    inputVelocity = modelCopy.inputVelocity;
    desiredVelocity = modelCopy.desiredVelocity;
    pose = modelCopy.pose;
    footprint.transformto(pose);
    dilatedFootprint.transformto(pose);
}

void Model::update(double dt, Communication &comm, bool setVelocity) {
    if(comm.newOdometry()) {
        pose = comm.measuredPose;
        measuredVelocity = comm.measuredVelocity;
        measuredVelocity.transformThis(0, 0, pose.a);
        Pose2D diff = velocity - measuredVelocity;
        if(abs(diff.a) > 0.3){velocity.a = measuredVelocity.a;}
        if(abs(diff.x) > 0.3){velocity.x = measuredVelocity.x;}
        if(abs(diff.y) > 0.3){velocity.y = measuredVelocity.y;}
    }

//    if(limitedMovements.size() > 0){cout << "Limit: ";}
//    for(auto & movement : limitedMovements){
//        switch(movement){
//            case Movement_Left: cout << " left "; break;
//            case Movement_Right: cout << " right "; break;
//            case Movement_Forward: cout << " forward "; break;
//            case Movement_Backward: cout << " backward "; break;
//            case Movement_RotateLeft:  cout << " rotate left "; break;
//            case Movement_RotateRight:  cout << " rotate right "; break;
//        }
//    }
//    if(limitedMovements.size() > 0){cout << endl;}

    updatePrediction(dt);

    if(setVelocity || applyBrake || !limitedMovements.empty()){
        Pose2D vel = inputVelocity;
        //Scaled velocity to overcome the static friction should be in the low level controller
        //Vector2D scaledVelLinear = vel.toVector() * (1 + ((maxSpeed - vel.length())/maxSpeed)*1);
        //double scaledVelAngular = vel.a * (1 + ((maxRotationalSpeed - vel.a)/maxRotationalSpeed)*1);
        //vel = Pose2D(scaledVelLinear, scaledVelAngular);
        vel.transformThis(0, 0, -pose.a);
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = vel.x;
        cmd_vel.linear.y = vel.y;
        cmd_vel.angular.z = vel.a;
        comm.setVel(cmd_vel);
    }else{
        desiredVelocity = Pose2D(0,0,0);
        inputVelocity = Pose2D(0,0,0);
    }
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

    //Pose2D minimalVelocity = origionalModel.velocity.unitPose()* (comm.minPredictionDistance_param/comm.predictionTime_param);

    for(int s = 0; s < comm.nTries_param; s++) {
        copyState(origionalModel);
        //if(velocity.length() < minimalVelocity.length()){velocity = minimalVelocity;}
        Vector2D prevPos = pose.toVector();
        double distance = 0;
        double N = ceil(comm.predictionTime_param / dt);
        Pose2D finalPredictionBias;
        int nLengthsAhead = 0;
        for (int p = 0; p < N; p++) {
            status = follow(tubes, comm, canvas, false);
            canvas.idName = "Prediction";
            updatePrediction(dt);
            distance += prevPos.distance(pose.toVector());
            prevPos = pose.toVector();
            finalPredictionBias = finalPredictionBias + predictionBiasVelocity * ((N-p)/N);
//            if(int(floor(distance / origionalModel.length())) == nLengthsAhead && distance <= brakeDistance()+brakeMargin+origionalModel.length()){
//                //show(canvas, Color(255,255,255), Thin);
//                checkCollision(comm.obstacles, canvas);
//                nLengthsAhead++;
//            }
            if (status != Status_Ok) { break; }
        }
        finalPredictionBias.constrainThis(velocity.toVector().length(), velocity.a);
        predictionBiasVelocity = finalPredictionBias;
        //show(canvas, Color(255, 255, 255), Thin);
        if (distance < comm.minPredictionDistance_param && status == Status_Ok) {
            status = Status_ShortPredictionDistance;
        }
        if ((status == Status_TubeCollision || status == Status_Stuck) && speedScale > minSpeedScale) {
            changeSpeedScale(speedScale*0.80);
        } else { break; }
    }
    //show(canvas, Color(255, 255, 255), Thin);
    return status;
}

void Model::showCommunicationInput(Visualization& canvas, Color c, int drawstyle, Communication &comm) {
    canvas.idName = "CommunicationInput";

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

