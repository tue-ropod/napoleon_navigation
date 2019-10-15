//
// Created by bob on 25-09-19.
//

#include "HolonomicModel.h"
#include <iostream>

HolonomicModel::HolonomicModel(Pose2D pose_, Polygon footprint_, double maxSpeed_, double maxAcceleration_, double wheelDistanceToMiddle_) :
    Model(pose_, std::move(footprint_), maxSpeed_, maxAcceleration_, wheelDistanceToMiddle_){
    velocity = Pose2D(0,0,0);
    inputVelocity = Pose2D(0,0,0);
    dilateFootprint(footprintClearance);
}

void HolonomicModel::input(Pose2D vel, Frame frame) {
    if(frame == Frame_World){
        vel.transformThis(0, 0, -pose.a + M_PI_2);
    }
    inputVelocity = vel;
    inputVelocity.transformThis(0, 0, -M_PI_2); //rotate object frame in world frame
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

void HolonomicModel::update(double dt) {

    Pose2D acc = (inputVelocity - velocity)/dt;
    if(acc.length() > maxAcceleration){
        Vector2D scaledAcc = acc.unit()*maxAcceleration;
        acc.x = scaledAcc.x;
        acc.y = scaledAcc.y;
    }

    if(abs(acc.a) > maxRotationalAcceleration){
        acc.a = (acc.a/abs(acc.a))*maxRotationalAcceleration;
    }

    velocity = velocity + acc * dt;

    if(velocity.length() > maxSpeed){
        Vector2D scaledVel = velocity.unit()*maxSpeed;
        velocity.x = scaledVel.x;
        velocity.y = scaledVel.y;
    }
    if(abs(velocity.a) > maxRotationalSpeed){
        velocity.a = (velocity.a/abs(velocity.a))*maxRotationalSpeed;
    }

    pose.rotateOrientation(velocity.a*dt);
    Vector2D transformedVelocity = velocity.transform(0, 0, pose.a);
    pose.transformThis(transformedVelocity.x*dt, transformedVelocity.y*dt, 0);
    footprint.transformto(pose);
    dilatedFootprint.transformto(pose);
}

void HolonomicModel::show(Visualization& canvas, Color c, int drawstyle) {
    canvas.polygon(footprint.vertices, c, drawstyle);
    canvas.polygon(dilatedFootprint.vertices, Color(255,0,0), Thin);

    Vector2D dir = velocity.toVector();
    dir = dir.transform(0, 0, pose.a).unit() * 0.5;
    dir.transformThis(pose.x, pose.y, 0);
    canvas.arrow(pose, dir, c, Thin);
}

FollowStatus HolonomicModel::follow(Tubes& tubes, int &tubeIndex, double speedScale, Visualization& canvas, bool debug){
    input(Pose2D(0,0,0)); //Stops automatically if it cannot follow the tubes
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
            int ti = tubes.tubeContainingPoint(vertex, tubeIndex);
            if(ti != -1){
                Tube &currentTube = tubes.tubes[ti];
                if(ti > tubeIndex) tubeIndex = ti;
                Vector2D pointVelocity;
                int ci = tubes.tubeCornerContainingPoint(vertex, tubeIndex);
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
                    pointVelocity = (r/tubeWidth) * maxSpeed * speedScale;
                }else{
                    pointVelocity = currentTube.velocity * maxSpeed * speedScale;
                }
                nVelocityVector++;
                velocityInput = velocityInput + translateInput(vertex, Pose2D(pointVelocity, 0));
                if(debug){canvas.arrow(vertex, vertex+pointVelocity/5, Color(0,0,255), Thin);}
                inTube = true;
            }
        }

        velocityInput = velocityInput/double(nVelocityVector);

        if( inTube ) {

            double collisionDistance = 0;

            int minTubeIndex = -1, maxTubeIndex = -1;
            for(auto & vertex : boundingBox){
                int ti = tubes.tubeContainingPoint(vertex, tubeIndex);
                if(ti != -1){
                    if(ti >= maxTubeIndex || maxTubeIndex == -1) maxTubeIndex = ti;
                    if(ti <= minTubeIndex || minTubeIndex == -1) minTubeIndex = ti;
                }
            }

            vector<Line> currentSides;
            for(int i = minTubeIndex; i <= maxTubeIndex; i++){ //Add tube sides to current sides that will be evaluated (mind the ordering left right)
                currentSides.emplace_back(tubes.tubes[i].leftSide);
                currentSides.emplace_back(tubes.tubes[i].rightSide);
            }

            bool leftSideBB = true;
            int nRepulsionVector = 0;
            int nCollision = 0;
            Pose2D repulsionInput;
            for(auto &sideBB : sidesBoundingBox){

                double minDist = -1;
                bool collision = false;
                Line collisionLine, collisionSide;
                for(int s = 0; s < currentSides.size(); s++){
                    Line &side = currentSides[s];
                    Line testLine = sideBB.shortestLineTo(side);
                    if(minDist == -1 || testLine.length() <= collisionLine.length()){
                        collisionSide = side;
                        Vector2D sideDir = (side.p2 - testLine.p2).unit();
                        if(s%2 == 0){sideDir.transformThis(0,0,M_PI_2);} //rightside
                        if(s%2 == 1){sideDir.transformThis(0,0,-M_PI_2);} //leftside
                        if(testLine.length() == 0){
                            collision = true;
                            collisionLine = Line(testLine.p1, testLine.p1 + sideDir);
                            break;
                        }else{
                            sideDir = sideDir * testLine.length();
                            collisionLine = Line(testLine.p1, testLine.p1 + sideDir);
                            minDist = collisionLine.length();
                        }
                    }
                }

                Vector2D repulsion = (collisionLine.p1 - collisionLine.p2);
                bool inside = false;
                for(int i = minTubeIndex; i <= maxTubeIndex; i++){ //Determine if the point closest to the side of the tube is inside the tube
                    inside = tubes.tubes[i].connectedShape.polygonContainsPoint(collisionLine.p1);
                    if(inside) break;
                }

                Vector2D pointVelocity, point;
                if(collision || !inside){ //Create a vector which pushes the Vertex which is outside the tube back
                    Vector2D vertex1 = sideBB.p1;
                    Vector2D vertex2 = sideBB.p2;
                    bool inTube1 = tubes.tubeContainingPoint(vertex1, minTubeIndex) != -1;
                    bool inTube2 = tubes.tubeContainingPoint(vertex2, minTubeIndex) != -1;
                    double dist = 0;
                    Vector2D point2;
                    if(inTube1 && !inTube2){
                        point = vertex2;
                        point2 = collisionSide.lineProjectionPoint(point);
                    }else if(!inTube1 && inTube2){
                        point = vertex1;
                        point2 = collisionSide.lineProjectionPoint(point);
                    }else if(inTube1 && inTube2){
                        point = sideBB.lineProjectionPointConstrained(collisionSide.p2);
                        repulsion = Vector2D(1,0);
                        if(leftSideBB) repulsion.transformThis(0, 0, pose.a + M_PI_2);
                        if(!leftSideBB) repulsion.transformThis(0, 0, pose.a - M_PI_2);
                        point2 = collisionSide.p2;
                    }else{
                        point = (vertex1 + vertex2)/2;
                        point2 = collisionSide.lineProjectionPoint(point);
                    }
                    dist = point.distance(point2);
                    pointVelocity = repulsion.unit() * maxSpeed * speedScale;
                    collisionDistance = collisionDistance + dist;
                    nCollision++;

                    if(debug){canvas.point(collisionLine.p1, Color(255, 0, 100), Thick);}
                    if(debug){canvas.line(collisionSide.p1, collisionSide.p2, Color(0,0,255), Thick);}
                    if(debug){canvas.line(point, point2, Color(255,255,255), Thin);}
                    if(debug){canvas.arrow(point, point+pointVelocity/5, Color(255,0,50), Thin);}
                }

                else if(inside) {
                    double dist = (0.2 - repulsion.length());
                    if (dist < 0) { dist = 0; }

                    int ci = tubes.tubeCornerContainingPoint(collisionLine.p1, minTubeIndex);
                    if(ci != -1){
                        Vector2D r = tubes.getCornerPoint(ci) - collisionLine.p1;
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
                        pointVelocity = (r/tubeWidth) * maxSpeed * speedScale * (dist);
                        point = collisionLine.p1;
                    }else {
                        int ti = tubes.tubeContainingPoint(collisionLine.p1, minTubeIndex);
                        point = collisionLine.p1;
                        pointVelocity = repulsion.unit() * maxSpeed * speedScale * (dist);
                        pointVelocity = pointVelocity + tubes.tubes[ti].velocity.unit() * maxSpeed * speedScale * (dist);
                    }

                    if(debug){canvas.arrow(point, point+pointVelocity, Color(0,255,50), Thin);}
                }

                repulsionInput = repulsionInput + translateInput(point, Pose2D(pointVelocity, 0));
                nRepulsionVector++;
                leftSideBB = false;
            }

            repulsionInput = repulsionInput/double(nRepulsionVector);
            Pose2D totalInput = velocityInput + repulsionInput;
            input(totalInput, Frame_World);

            if(collisionDistance > footprintClearance){
                status = Status_Collision;
            }else if(tubeIndex == tubes.tubes.size()-1){
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
