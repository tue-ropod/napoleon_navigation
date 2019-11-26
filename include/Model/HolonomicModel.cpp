//
// Created by bob on 25-09-19.
//

#include "HolonomicModel.h"
#include <iostream>

HolonomicModel::HolonomicModel(Pose2D pose_, Polygon footprint_, double maxSpeed_, double maxAcceleration_, double wheelDistanceToMiddle_) :
    Model(pose_, std::move(footprint_), maxSpeed_, maxAcceleration_, wheelDistanceToMiddle_){
    dilateFootprint(footprintClearance);
}

void HolonomicModel::input(Pose2D vel, Frame frame) {
    switch(frame){
        case Frame_Robot:
            vel.transformThis(0, 0, pose.a);
            break;
        case Frame_World:
            break;
    }
    desiredVelocity = vel;
}

Pose2D HolonomicModel::translateInput(Vector2D position, Pose2D v_p) {
    Vector2D r_p = pose.toVector() - position;
    double radius = r_p.length();
    if(radius > 0.0001) {
        Vector2D rotationVector = r_p.transform(0, 0, v_p.a) - r_p;
        Vector2D vLongitudinal = v_p.transform(0, 0, -r_p.angle());
        double vTangential = -vLongitudinal.length() * sin(vLongitudinal.angle());
        double vRadial = vLongitudinal.length() * cos(vLongitudinal.angle());
        double omega = (vTangential / radius) + v_p.a;
        Vector2D velocityVector = rotationVector + r_p.unit() * vRadial;
        //Vector2D velocityVector = rotationVector + v_p;
        Pose2D transformedVelocity(velocityVector.x, velocityVector.y, omega);
        return transformedVelocity;
    }else{
        return v_p;
    }
}

void HolonomicModel::updatePrediction(double dt) {
    if(applyBrake){
        inputVelocity = Pose2D(0,0,0);
        applyBrake = false;
    }else{
        Pose2D desiredAcceleration = (desiredVelocity - velocity)/dt;
        desiredAcceleration.constrainThis(maxAcceleration, maxRotationalAcceleration);
        inputVelocity = velocity + desiredAcceleration * dt;
        inputVelocity.constrainThis(maxSpeed, maxRotationalSpeed);
        inputVelocity.transformThis(0,0,-pose.a);
        for(auto & movement : limitedMovements){
            switch(movement){
                case Movement_Left: if(inputVelocity.y < 0){inputVelocity.y = 0;} break;
                case Movement_Right: if(inputVelocity.y > 0){inputVelocity.y = 0;} break;
                case Movement_Forward: if(inputVelocity.x > 0){inputVelocity.x = 0;} break;
                case Movement_Backward: if(inputVelocity.x < 0){inputVelocity.x = 0;} break;
                case Movement_RotateLeft: if(inputVelocity.a > 0){inputVelocity.a = 0;} break;
                case Movement_RotateRight: if(inputVelocity.a < 0){inputVelocity.a = 0;} break;
            }
        }
        inputVelocity.transformThis(0,0,pose.a);
    }
    limitedMovements.clear();
    velocity = inputVelocity;
    updateModel(dt);
}

void HolonomicModel::updateModel(double dt) {
    pose.rotateOrientation(velocity.a*dt);
    pose.transformThis(velocity.x*dt, velocity.y*dt, 0);
    footprint.transformto(pose);
    dilatedFootprint.transformto(pose);
}

FollowStatus HolonomicModel::follow(Tubes& tubes, Communication& comm, Visualization& canvas, bool debug){
    status = Status_Error; //Pre set status to error
    canvas.idName = "Follow Debug";

    if(!tubes.tubes.empty() && !dilatedFootprint.vertices.empty()){
        Pose2D velocityInput;

        vector<Vector2D> boundingBox = dilatedFootprint.boundingBoxRotated(pose.a);
        vector<Vector2D> frontBoundingBox = {boundingBox[1], boundingBox[2]};
        vector<Line> sidesBoundingBox = {Line(boundingBox[0],boundingBox[1]), Line(boundingBox[3],boundingBox[2])};
        //vector<Vector2D> midBoundingBox = {(boundingBox[1]+boundingBox[0])/2, (boundingBox[2]+boundingBox[3])/2};
        //vector<Vector2D> frontFootprint = {footprint.vertices[2], footprint.vertices[3]};
        //vector<Line> sidesFootprint = {Line(footprint.vertices[0],footprint.vertices[1]),Line(footprint.vertices[5],footprint.vertices[4])};

        currentTubeIndex = tubes.tubeContainingPoint(pose, currentTubeIndex);

        int nVelocityVector = 0;
        for(auto & vertex : frontBoundingBox){
            int ti = tubes.tubeContainingPoint(vertex, currentTubeIndex);
            if(ti != -1){
                Tube &currentTube = tubes.tubes[ti];
                Vector2D pointVelocity;
                int ci = tubes.tubeCornerContainingPoint(vertex, currentTubeIndex);
                if(ci != -1){ //footprint point in corner
                    Vector2D r = tubes.getCornerPoint(ci) - vertex;
                    switch(tubes.getCornerSide(ci)){
                        case Corner_Left:
                            r.transformThis(0,0,-M_PI_2);
                            break;
                        case Corner_Right:
                            r.transformThis(0,0,M_PI_2);
                            break;
                        case Corner_None:
                            break;
                    }
                    double tubeWidth = tubes.tubes[ci].width2;
                    pointVelocity = (r/tubeWidth) * maxSpeed * speedScale * currentTube.velocity.length() * sqrt(2);
                }else{ //straight ahead
                    pointVelocity = currentTube.velocity * maxSpeed * speedScale;
                }
                nVelocityVector++;
                velocityInput = velocityInput + translateInput(vertex, Pose2D(pointVelocity, 0));
                if(debug){canvas.arrow(vertex, vertex+pointVelocity, Color(0,0,255), Thin);}
            }
        }

        velocityInput = velocityInput/double(nVelocityVector);

        if( currentTubeIndex != -1 ) { //Middle of object inside tube

            //Calculate the minimum and maximum tube index which the footprint of the object occupies
            int minTubeIndex = -1, maxTubeIndex = -1;
            for(auto & vertex : boundingBox){
                int ti = tubes.tubeContainingPoint(vertex, currentTubeIndex);
                if(ti != -1){
                    if(ti >= maxTubeIndex || maxTubeIndex == -1) maxTubeIndex = ti;
                    if(ti <= minTubeIndex || minTubeIndex == -1) minTubeIndex = ti;
                }
            }

            //create a vector of the current sides of the occupied tubes
            vector<Line> currentSides;
            vector<int> sideIndex;
            for(int i = minTubeIndex; i <= maxTubeIndex; i++){ //Add tube sides to current sides that will be evaluated (mind the ordering left > right)
                currentSides.emplace_back(tubes.tubes[i].leftSide);
                currentSides.emplace_back(tubes.tubes[i].rightSide);
                sideIndex.emplace_back(i);
                sideIndex.emplace_back(i);
            }

            double border = comm.tubeExtraSpace_param/2;
            double collisionDistance = 0;
            int nRepulsion = 0;
            int nCollision = 0;
            Pose2D repulsionInput, collisionInput;

            for(auto &vertex : boundingBox){
                int Tp = tubes.tubeContainingPoint(vertex, currentTubeIndex);
                int Cp = tubes.tubeCornerContainingPoint(vertex, currentTubeIndex);

                if(Tp != -1){ //inside tube
                    vector<Line> tubeSides;
                    tubeSides.emplace_back(tubes.tubes[Tp].leftSide);
                    tubeSides.emplace_back(tubes.tubes[Tp].rightSide);
                    for(auto &tubeSide : tubeSides){
                        Line testLine = Line(vertex, tubeSide.lineProjectionPointConstrained(vertex));
                        if(testLine.length() <= border){
                            double p = (border - testLine.length())/border; //1 when collision 0 when at the border
                            Vector2D sideDirection = (testLine.p1 - testLine.p2).unit(); //create perpendicular vector to side
                            Vector2D pointVelocity;
                            if(Cp == -1){//vertex not in corner
                                //create perpendicular velocity vector
                                pointVelocity = sideDirection * velocityInput.length() * p;

                            }else{//vertex in corner
                                //combine perpendicular side vector with tube velocity
                                if (debug) { canvas.point(vertex, Color(0, 255, 255), Thick); }
                                Vector2D tubeVelocityDirection = tubes.tubes[Tp].velocity.unit();
                                pointVelocity = (sideDirection*p + tubeVelocityDirection*(1-p)) * velocityInput.length();
                            }
                            //add velocity vector to repulsion vectors
                            repulsionInput = repulsionInput + translateInput(vertex, Pose2D(pointVelocity, 0));
                            nRepulsion++;
                            if (debug) { canvas.arrow(vertex, vertex+pointVelocity, Color(255, 0, 255), Thin); }
                        }
                    }
                }else{ //outside tube
                    //calculate closest tube side
                    Line closestLine;
                    bool found = false;
                    for(auto &tubeSide : currentSides){
                        Line testLine = Line(vertex, tubeSide.lineProjectionPointConstrained(vertex));
                        if(testLine.length() < closestLine.length() || !found){
                            closestLine = testLine;
                            found = true;
                        }
                    }
                    Vector2D sideDirection = (closestLine.p2 - closestLine.p1).unit(); //create perpendicular vector to closest side
                    Vector2D pointVelocity = sideDirection * velocityInput.length(); //create velocity vector
                    //add velocity vector to collision vectors
                    collisionInput = collisionInput + translateInput(vertex, Pose2D(pointVelocity, 0));
                    nCollision++;
                    collisionDistance += closestLine.length();
                    if (debug) { canvas.arrow(vertex, vertex+pointVelocity, Color(255, 0, 0), Thin); }
                }
            }

            for(auto &sideBB : sidesBoundingBox){
                Line closestLine;
                bool found = false;
                for(int ti = minTubeIndex; ti <= maxTubeIndex; ti++){
                    Vector2D cornerPoint = tubes.getCornerPoint(ti);
                    if(cornerPoint.valid()){
                        Line testLine = Line(cornerPoint, sideBB.lineProjectionPointConstrained(cornerPoint));
                        if(testLine.length() < closestLine.length() || !found){
                            closestLine = testLine;
                            found = true;
                        }
                    }
                }
                if(found){
                    int Tp = tubes.tubeContainingPoint(closestLine.p2, currentTubeIndex);
                    if(Tp != -1){ //inside tube
                        if(closestLine.length() <= border){
                            double p = (border - closestLine.length())/border; //1 when collision 0 when at the border
                            Vector2D direction = (closestLine.p2 - closestLine.p1).unit(); //create perpendicular vector to side
                            Vector2D pointVelocity = direction * velocityInput.length() * p;
                            //add velocity vector to repulsion vectors
                            repulsionInput = repulsionInput + translateInput(closestLine.p2, Pose2D(pointVelocity, 0));
                            nRepulsion++;
                            if (debug) { canvas.arrow(closestLine.p2, closestLine.p2+pointVelocity, Color(255, 0, 255), Thin); }
                        }
                    }else{ //ouside tube
                        Vector2D direction = (closestLine.p1 - closestLine.p2).unit(); //create perpendicular vector to side
                        Vector2D pointVelocity = direction * velocityInput.length();
                        collisionInput = collisionInput + translateInput(closestLine.p2, Pose2D(pointVelocity, 0));
                        nCollision++;
                        collisionDistance += closestLine.length();
                        if (debug) { canvas.arrow(closestLine.p2, closestLine.p2+pointVelocity, Color(255, 0, 0), Thin); }
                    }
                }
            }

            if(nRepulsion > 0){repulsionInput = repulsionInput/double(nRepulsion);}
            if(nCollision > 0){collisionInput = collisionInput/double(nCollision);}
            Pose2D currentCorrectionInput = (repulsionInput*2 + collisionInput*2)/2; //TODO determine repulsion and collision factor
            Pose2D correctionInput = currentCorrectionInput*0.8 + predictionBiasVelocity*0.2;
            Pose2D totalInput = velocityInput + correctionInput;
            predictionBiasVelocity = correctionInput;
            input(totalInput, Frame_World);

            double inputDirectionDifference = (totalInput.toVector().angle() - velocityInput.toVector().angle());
            smallestAngle(inputDirectionDifference);
            double directiondifference = (pose.a - velocityInput.toVector().angle());
            smallestAngle(directiondifference);

            if(collisionDistance > footprintClearance){
                status = Status_TubeCollision;
            }else if(tubes.tubes.size()-1 == currentTubeIndex){
                status = Status_Done;
            }else if(abs(inputDirectionDifference) > M_PI_4*3){
                status = Status_Stuck;
            }else if(abs(directiondifference) > 2*M_PI/3){
                status = Status_WrongWay;
            }else{
                status = Status_Ok;
            }
        }else{ //Middle not in tube
            if((prevStatus == Status_OutsideTube && velocity.length() < 0.01) || prevStatus == Status_Recovering){
                status = Status_Recovering;
            }else{
                status = Status_OutsideTube;
            }
        }
    }
    return status;
}

void HolonomicModel::show(Visualization& canvas, Color c, int drawstyle) {
    canvas.idName = "HolonomicModel";

    canvas.polygon(footprint.vertices, c, drawstyle);
    canvas.polygon(dilatedFootprint.vertices, Color(255,0,0), Thin);
    canvas.arrow(pose, velocity.toVector()*1+pose, Color(0,0,255), Thin);
    //canvas.arrow(pose, inputVelocity.toVector()*1+pose, Color(255,0,255), Thin);
    //canvas.arrow(pose, predictionBiasVelocity.toVector()*1+pose, Color(0,255,0), Thin);

    Vector2D offset = Vector2D(1,0).transform(0,0,pose.a).transform(pose.x,pose.y,0);
    Vector2D dir = Vector2D(0,1).transform(0,0,pose.a)*velocity.a;
    canvas.arrow(offset, offset+dir, Color(0,0,255), Thin);
}