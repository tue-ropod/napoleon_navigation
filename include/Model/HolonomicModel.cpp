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
    }else{
        calculateInputVelocity(dt);
    }
    velocity = inputVelocity;
    updateModel(dt);
}

void HolonomicModel::updateModel(double dt) {
    pose.rotateOrientation(velocity.a*dt);
    pose.transformThis(velocity.x*dt, velocity.y*dt, 0);
    footprint.transformto(pose);
    dilatedFootprint.transformto(pose);
}

FollowStatus HolonomicModel::follow(Tubes& tubes, Visualization& canvas, bool debug){
    input(Pose2D(0,0,0), Frame_Robot); //Stops automatically if it cannot follow the tubes
    FollowStatus status = Status_Error; //Pre set status to error

    if(!tubes.tubes.empty() && !dilatedFootprint.vertices.empty()){
        bool inTube = false;
        Pose2D velocityInput;

        vector<Vector2D> boundingBox = dilatedFootprint.boundingBoxRotated(pose.a);
        vector<Vector2D> frontBoundingBox = {boundingBox[1], boundingBox[2]};
        vector<Vector2D> midBoundingBox = {(boundingBox[1]+boundingBox[0])/2, (boundingBox[2]+boundingBox[3])/2};
        vector<Line> sidesBoundingBox = {Line(boundingBox[0],boundingBox[1]), Line(boundingBox[3],boundingBox[2])};

        //vector<Vector2D> frontFootprint = {footprint.vertices[2], footprint.vertices[3]};
        //vector<Line> sidesFootprint = {Line(footprint.vertices[0],footprint.vertices[1]),Line(footprint.vertices[5],footprint.vertices[4])};

        int nVelocityVector = 0;
        for(auto & vertex : frontBoundingBox){
            int ti = tubes.tubeContainingPoint(vertex, currentTubeIndex);
            if(ti != -1){
                Tube &currentTube = tubes.tubes[ti];
                if(ti > currentTubeIndex) currentTubeIndex = ti;
                Vector2D pointVelocity;
                int ci = tubes.tubeCornerContainingPoint(vertex, currentTubeIndex);
                if(ci != -1){
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
                    pointVelocity = (r/tubeWidth) * maxSpeed * speedScale * currentTube.velocity.length();
                }else{
                    pointVelocity = currentTube.velocity * maxSpeed * speedScale;
                }
                nVelocityVector++;
                velocityInput = velocityInput + translateInput(vertex, Pose2D(pointVelocity, 0));
                if(debug){canvas.arrow(vertex, vertex+pointVelocity, Color(0,0,255), Thin);}
                inTube = true;
            }
        }

        velocityInput = velocityInput/double(nVelocityVector);

        if( inTube ) {

            int minTubeIndex = -1, maxTubeIndex = -1;
            for(auto & vertex : boundingBox){
                int ti = tubes.tubeContainingPoint(vertex, currentTubeIndex);
                if(ti != -1){
                    if(ti >= maxTubeIndex || maxTubeIndex == -1) maxTubeIndex = ti;
                    if(ti <= minTubeIndex || minTubeIndex == -1) minTubeIndex = ti;
                }
            }

            vector<Line> currentSides;
            vector<int> sideIndex;
            for(int i = minTubeIndex; i <= maxTubeIndex; i++){ //Add tube sides to current sides that will be evaluated (mind the ordering left > right)
                currentSides.emplace_back(tubes.tubes[i].leftSide);
                currentSides.emplace_back(tubes.tubes[i].rightSide);
                sideIndex.emplace_back(i);
                sideIndex.emplace_back(i);
            }

            double border = 0.1;
            double collisionDistance = 0;
            int nRepulsion = 0;
            int nCollision = 0;
            Pose2D repulsionInput, collisionInput;
            for(auto &sideBB : sidesBoundingBox) {
                bool leftSide = true;
                for (int s = 0; s < currentSides.size(); s++) {
                    Line &side = currentSides[s];
                    int currentTubeIndex = sideIndex[s];
                    Line testLine = sideBB.shortestLineTo(side);
                    Vector2D point = testLine.p1;
                    Vector2D pointVelocity;
                    Vector2D currentTubeVelocity = tubes.tubes[currentTubeIndex].velocity;
                    int Tp = tubes.tubeContainingPoint(point, currentTubeIndex);
                    //int Cp = tubes.tubeCornerContainingPoint(point, currentTubeIndex);

                    if (testLine.length() == 0) { // Intersection > Collision
                        Line collisionLine;
                        bool valid = false;
                        int p1_inside = tubes.tubeContainingPoint(sideBB.p1, currentTubeIndex);
                        int p2_inside = tubes.tubeContainingPoint(sideBB.p2, currentTubeIndex);
                        if(p1_inside == -1){
                            if(leftSide){collisionLine = Line(sideBB.p1, tubes.tubes[currentTubeIndex].leftSide.lineProjectionPoint(sideBB.p1));}
                            else{collisionLine = Line(sideBB.p1, tubes.tubes[currentTubeIndex].rightSide.lineProjectionPoint(sideBB.p1));}
                            collisionDistance += collisionLine.length();
                            valid = true;
                        }else if(p2_inside == -1){
                            if(leftSide){collisionLine = Line(sideBB.p2, tubes.tubes[currentTubeIndex].leftSide.lineProjectionPoint(sideBB.p2));}
                            else{collisionLine = Line(sideBB.p2, tubes.tubes[currentTubeIndex].rightSide.lineProjectionPoint(sideBB.p2));}
                            collisionDistance += collisionLine.length();
                            valid = true;
                        }else{
                            Vector2D cornerPoint = tubes.getCornerPoint(p1_inside); //p1 is the back in both cases
                            if(cornerPoint.valid()){
                                collisionLine = Line(sideBB.lineProjectionPoint(cornerPoint), cornerPoint);
                                collisionDistance += collisionLine.length()/2; //add half because this is valid 2 times
                                valid = true;
                            }
                        }
                        if(valid){
                            point = collisionLine.p1;
                            pointVelocity = (collisionLine.p2 - collisionLine.p1).unit() * maxSpeed * speedScale; // Not multiplied with tube velocity
                            collisionInput = collisionInput + translateInput(point, Pose2D(pointVelocity, 0));
                            nCollision++;
                            if (debug) { canvas.point(point, Color(255, 0, 0), Thin); }
                            if (debug) { canvas.arrow(point, point+pointVelocity, Color(255, 0, 0), Thin); }
                        }
//                        pointVelocity = (side.p2 - testLine.p1).unit();
//                        if (leftSide) { pointVelocity.transformThis(0, 0, -M_PI_2); } //leftside
//                        if (!leftSide) { pointVelocity.transformThis(0, 0, M_PI_2); } //rightside
//                        pointVelocity = pointVelocity.unit() * maxSpeed * speedScale * currentTubeVelocity.length();

                    }else if(Tp == -1){ //Fully outside > Collision
                        collisionDistance += testLine.length();
                        pointVelocity = (testLine.p2 - testLine.p1).unit() * maxSpeed * speedScale; // Not multiplied with tube velocity
                        collisionInput = collisionInput + translateInput(point, Pose2D(pointVelocity, 0));
                        nCollision++;
                        if (debug) { canvas.arrow(point, point+pointVelocity, Color(255, 0, 255), Thin); }

                    }else if (testLine.length() <= border) { //Inside > repulsion
//                        if(Cp != -1){
//                            Vector2D r = (tubes.getCornerPoint(Cp) - point);
//                            switch(tubes.getCornerSide(Cp)){
//                                case Corner_Left:
//                                    r.transformThis(0,0,-M_PI_2);
//                                    break;
//                                case Corner_Right:
//                                    r.transformThis(0,0,M_PI_2);
//                                    break;
//                                case Corner_None:
//                                    break;
//                            }
//                            double tubeWidth = tubes.tubes[Cp].width2;
//                            currentTubeVelocity = (r/tubeWidth) * maxSpeed * speedScale * currentTubeVelocity.length();
//                        }
                        double p = (border - testLine.length())/border; //1 when collision 0 when at the border
                        double p2 = p*p;
                        Vector2D repellantVelocity = (testLine.p1 - testLine.p2).unit() * maxSpeed * speedScale; // Not multiplied with tube velocity
                        pointVelocity = ((repellantVelocity*p2 + currentTubeVelocity*(1-p2)))*p;
                        repulsionInput = repulsionInput + translateInput(point, Pose2D(pointVelocity, 0));
                        nRepulsion++;
                        if (debug) { canvas.arrow(point, point+pointVelocity, Color(0, 255, 0), Thin); }
                    }
                    leftSide = !leftSide;
                }
            }

            if(nRepulsion > 0){repulsionInput = repulsionInput/double(nRepulsion);}
            if(nCollision > 0){collisionInput = collisionInput/double(nCollision);}
            Pose2D correctionInput = (repulsionInput + collisionInput)/2;
            Pose2D totalInput = velocityInput + (correctionInput + predictionBiasVelocity)/2;
            predictionBiasVelocity = correctionInput;
            input(totalInput, Frame_World);

            if(collisionDistance > footprintClearance){ // TODO calculate collision distance
                status = Status_Collision;
            }else if(tubes.tubes[tubes.tubes.size()-1].connectedShape.polygonContainsPoint(pose)){
                status = Status_Done;
            }else if(totalInput.length()<0.01 && abs(totalInput.a) < 0.01){ //TODO Determine when stuck
                status = Status_Stuck;
            }else{
                status = Status_Ok;
            }
        }
    }
    return status;
}

void HolonomicModel::show(Visualization& canvas, Color c, int drawstyle) {
    canvas.polygon(footprint.vertices, c, drawstyle);
    canvas.polygon(dilatedFootprint.vertices, Color(255,0,0), Thin);
    canvas.arrow(pose, velocity.toVector()*1+pose, Color(0,0,255), Thin);
    //canvas.arrow(pose, inputVelocity.toVector()*1+pose, Color(255,0,255), Thin);
    //canvas.arrow(pose, predictionBiasVelocity.toVector()*1+pose, Color(0,255,0), Thin);

    Vector2D offset = Vector2D(1,0).transform(0,0,pose.a).transform(pose.x,pose.y,0);
    Vector2D dir = Vector2D(0,1).transform(0,0,pose.a)*velocity.a;
    canvas.arrow(offset, offset+dir, Color(0,0,255), Thin);
}