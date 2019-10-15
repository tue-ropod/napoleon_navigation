//
// Created by bob on 02-10-19.
//

#include "Tubes.h"

Tubes::Tubes(const Tube& tube){
    tubes.emplace_back(tube);
}

void Tubes::addPoint(Vector2D p, double width, double speed, int index){
    if(!tubes.empty()) {
        double angle = M_PI;
        if(index > 0 && index < tubes.size()){
            //TODO more than 90 degree turns add point (or find a different way to combine the sides of the tubes)
        }
        if(angle < M_PI_2){
            Vector2D newP = p;
            newP = newP; //TODO determine newP
            addPoint(newP, width, speed, index);
            addPoint(p, width, speed, index+1);
        }else{
            if (index < 0) {
                Tube &lastTube = tubes[tubes.size() - 1];
                Tube tube(lastTube.p2, lastTube.width2, p, width, speed);
                tubes.emplace_back(tube);
                connectTubes(tubes.size() - 2);
            } else if (index > 0 && index < tubes.size()) {
                Tube &r = tubes[index];
                r.buildTube(r.p1, r.width1, p, width, speed);
                Tube n = tubes[index + 1];
                Tube tube(p, width, n.p1, n.width1, speed);
                tubes.insert(tubes.begin() + index + 1, tube);
                connectTubes(index - 1);
                connectTubes(index);
                connectTubes(index + 1);
            }
        }
    }
}

void Tubes::removePoint(unsigned int index) {
    if(index <= tubes.size()) {
        if(index > 0 && index < tubes.size()){
            tubes.erase(tubes.begin()+index);
            Tube &nextTube = tubes[index];
            Tube &prevTube = tubes[index-1];
            prevTube.buildTube(prevTube.p1, prevTube.width1, nextTube.p1, nextTube.width1, prevTube.velocity.length());
            connectTubes(index-2);
            connectTubes(index-1);
        }else{ //Remove first or last tube no rebuilding needed
            if(index == tubes.size()) index--;
            tubes.erase(tubes.begin()+index);
        }
    }
}

void Tubes::avoidObstacles(unsigned int startIndex, unsigned int index, vector<Obstacle>& obstacles, Model& model, DrivingSide side, Visualization &canvas){
    if(index < tubes.size() && !obstacles.empty()) {
        Tube &tube = tubes[index];
        bool obstruction = false;
        for (auto &obstacle : obstacles){
            //TODO Add scan window (width of hallway X forward)
            if(tube.connectedShape.polygonContainsPoint(obstacle.footprint.middle) || tube.connectedShape.polygonPolygonCollision(obstacle.footprint) ){
                obstruction = true;
                break;
            }
        }
        if( obstruction ){
            Line direction(tube.p1, tube.p2);
            vector<Vector2D> modelProjection = model.footprint.projectPolygonOnLine(direction);
            Vector2D frontPoint = modelProjection[1];
            //canvas.point(frontPoint, Color(0, 255, 255), Thick);

            Vector2D obstacleProjectionDir = Vector2D(tube.p2 - tube.p1);
            obstacleProjectionDir.transformThis(0, 0, -M_PI_2);
            Line obstacleProjectionLine(tube.p1, tube.p1 + obstacleProjectionDir);

            bool newTubePlace = false;
            Vector2D closestObstaclePoints[4];
            double closestDistance = -1;
            Obstacle *closestObstacle = nullptr;

            for (auto &obstacle : obstacles){
                if(tube.connectedShape.polygonContainsPoint(obstacle.footprint.middle) || tube.connectedShape.polygonPolygonCollision(obstacle.footprint) ){

                    vector<Vector2D> obstacleProjection = obstacle.footprint.projectPolygonOnLine(obstacleProjectionLine);
                    vector<double> distances = obstacle.footprint.distancePolygonToLineMinMax(obstacleProjectionLine);
                    Vector2D distanceMinVector = (direction.p2 - direction.p1).unit() * distances[0];
                    Vector2D distanceMaxVector = (direction.p2 - direction.p1).unit() * distances[1];
                    //canvas.line(obstacleProjection[0]+distanceMinVector, obstacleProjection[1]+distanceMinVector, Color(255,255,255), Thin);
                    //canvas.line(obstacleProjection[0]+distanceMaxVector, obstacleProjection[1]+distanceMaxVector, Color(255,255,255), Thin);

                    if( distances[0]>=0 && (distances[0]<=closestDistance || closestDistance == -1) ){
                        closestObstaclePoints[0] = obstacleProjection[0] + distanceMinVector; //Bottom Left
                        closestObstaclePoints[1] = obstacleProjection[1] + distanceMinVector; //Bottom Right
                        closestObstaclePoints[2] = obstacleProjection[0] + distanceMaxVector; //Top Left
                        closestObstaclePoints[3] = obstacleProjection[1] + distanceMaxVector; //Top Right
                        closestDistance = distances[0];
                        closestObstacle = &obstacle;
                        newTubePlace = true;
                    }
                }
            }
            if( newTubePlace ) {
                Vector2D newTubePoint1, newTubePoint2;
                switch (side) {
                    case DrivingSide_Right:
                        newTubePoint1 = closestObstaclePoints[0] + obstacleProjectionDir.unit() * -tube.width2 / 1.9;
                        newTubePoint2 = closestObstaclePoints[2] + obstacleProjectionDir.unit() * -tube.width2 / 1.9;
                        break;
                    case DrivingSide_Left:
                        newTubePoint1 = closestObstaclePoints[1] + obstacleProjectionDir.unit() * tube.width2 / 1.9;
                        newTubePoint2 = closestObstaclePoints[3] + obstacleProjectionDir.unit() * tube.width2 / 1.9;
                        break;
                    case DrivingSide_Automatic:
                        //TODO choose best direction
                        cout << "CHOOSE DIRECTION" << endl;
                }
                //canvas.line(newTubePoint1, newTubePoint2, Color(255, 0, 255), Thick);
                Circle tubeConnectionArea((tube.leftSide.p2+tube.rightSide.p2)/2, ((tube.leftSide.p2-tube.rightSide.p2)/2).length());
                if(closestObstacle!=nullptr && closestObstacle->footprint.polygonInCircle(tubeConnectionArea)){
                    removePoint(index+1);
                }
                //addPoint(frontPoint, currentTube.width2, currentTube.velocity.length(), index);
                addPoint(newTubePoint1, tube.width2, tube.velocity.length(), index + 0);
                addPoint(newTubePoint2, tube.width2, tube.velocity.length(), index + 1);

                avoidObstacles(startIndex, index+2, obstacles, model, side, canvas);
            }
        }else if(index < startIndex+1){
            avoidObstacles(startIndex, index+1, obstacles, model, side, canvas);
        }
    }
}

void Tubes::connectTubes(unsigned int index){
    if(index < tubes.size()-1) {
        Tube &tube1 = tubes[index];
        Tube &tube2 = tubes[index+1];
        tube1.resetSides(End);
        tube2.resetSides(Begin);
        Vector2D newPointLeft = tube1.leftSide.lineLineIntersection(tube2.leftSide);
        Vector2D newPointRight = tube1.rightSide.lineLineIntersection(tube2.rightSide);
        if(isnan(newPointLeft.x)){
            newPointLeft = (tube1.leftSide.p2 + tube2.leftSide.p1)/2;
        }
        if(isnan(newPointRight.x)){
            newPointRight = (tube1.rightSide.p2 + tube2.rightSide.p1)/2;
        }
        tube1.setSides(End, newPointLeft, newPointRight);
        tube2.setSides(Begin, newPointLeft, newPointRight);
    }
}

int Tubes::tubeContainingPoint(Vector2D& point, int initialSearchPoint){
    int searchvalue = 0;
    bool lb = false, ub = false;

    while(true){
        int i = initialSearchPoint+searchvalue;
        if(i < 0) lb = true;
        else if(i >= tubes.size()) ub = true;
        else{
            Tube &testtube = tubes[i];
            if(testtube.connectedShape.polygonContainsPoint(point)){
                return i;
            }
        }

        if(lb && ub) break; // break if index out of bounds

        //search around the offset using searchvalue [0 1 -1 2 -2 3 -3 ...]
        if(searchvalue == 0) searchvalue = 1;
        else{
            if(searchvalue < 0) searchvalue--;
            searchvalue *= -1;
        }
    }
    return -1;
}

int Tubes::tubeCornerContainingPoint(Vector2D& point, int initialSearchPoint){
    int searchvalue = 0;
    bool lb = false, ub = false;

    while(true){
        int i = initialSearchPoint+searchvalue;
        if(i < 0) lb = true;
        else if(i >= tubes.size()) ub = true;
        else{
            if(getCornerArea(i).polygonContainsPoint(point)){
                return i;
            }
        }

        if(lb && ub) break; // break if index out of bounds

        //search around the offset using searchvalue [0 1 -1 2 -2 3 -3 ...]
        if(searchvalue == 0) searchvalue = 1;
        else{
            if(searchvalue < 0) searchvalue--;
            searchvalue *= -1;
        }
    }
    return -1;
}

Corner Tubes::getCornerSide(unsigned int index) {
    Corner side = Corner_None;
    if(index < tubes.size()-2) {
        Vector2D v1 = tubes[index].p2 - tubes[index].p1;
        Vector2D v2 = tubes[index+1].p2 - tubes[index+1].p1;
        v2.transformThis(0,0,-v1.angle());
        if(v2.angle() <= 0){ //right turn
            side = Corner_Right;
        }else{
            side = Corner_Left;
        }
    }
    return side;
}

Vector2D Tubes::getCornerPoint(unsigned int index) {
    Vector2D cornerPoint;
    switch (getCornerSide(index)){
        case Corner_Left:
            cornerPoint = tubes[index].leftSide.p2;
            break;
        case Corner_Right:
            cornerPoint = tubes[index].rightSide.p2;
            break;
        case Corner_None:
            cornerPoint = Vector2D(NAN, NAN);
            break;
    }
    return cornerPoint;
}

Polygon Tubes::getCornerArea(unsigned int index){
    Polygon cornerArea;
    if(index < tubes.size()-2) {
        Vector2D c = getCornerPoint(index);
        Vector2D p1, p2, p3, p4;
        switch(getCornerSide(index)){
            case Corner_Left:
                p1 = c;
                p2 = tubes[index].rightSide.lineProjectionPoint(c);
                p3 = tubes[index].rightSide.p2;
                p4 = tubes[index+1].rightSide.lineProjectionPoint(c);
                break;
            case Corner_Right:
                p1 = c;
                p2 = tubes[index+1].leftSide.lineProjectionPoint(c);
                p3 = tubes[index].leftSide.p2;
                p4 = tubes[index].leftSide.lineProjectionPoint(c);
                break;
            case Corner_None:
                break;
        }
        cornerArea = Polygon({p1, p2, p3, p4});
    }
    return cornerArea;
}

void Tubes::showOriginalTubes(Visualization &canvas) {
    for(auto & tube : tubes){
        tube.showOriginalTube(canvas, Color(255*(1-tube.velocity.length()),255*(tube.velocity.length()),0), Thin);
    }
}

void Tubes::showTubes(Visualization &canvas) {
    for(auto & tube : tubes){
        tube.showTube(canvas, Color(255*(1-tube.velocity.length()),255*(tube.velocity.length()),0), Thin);
    }
}

void Tubes::showSides(Visualization &canvas) {
    for(auto & tube : tubes){
        tube.showSides(canvas, Color(255*(1-tube.velocity.length()),255*(tube.velocity.length()),0), Thin);
    }
    for(unsigned int i = 0; i < tubes.size(); i++){
        canvas.point(getCornerPoint(i), Color(255,0,0), Thick);
        canvas.polygon(getCornerArea(i).vertices, Color(100,100,200), Thin);
    }
}


