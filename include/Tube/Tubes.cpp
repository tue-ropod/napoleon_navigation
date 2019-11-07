//
// Created by bob on 02-10-19.
//

#include "Tubes.h"

Tubes::Tubes(const Tube& tube){
    tubes.emplace_back(tube);
}

bool Tubes::convertRoute(ropod_ros_msgs::RoutePlannerResult &route, Model &model, Visualization &canvas) {
    std::vector<ropod_ros_msgs::Area> &areas = route.areas;
    tubes.clear();
    bool fit = true;
    double extraSpace = 0.4;
    double wallOffset = 0.2;
    double corridorSpeed = 1;
    double junctionSpeed = 0.5;
    DrivingSide drivingSide = DrivingSide_Right; //TODO change tube generation based on driving side now only right side is supported

    bool firstTubePlaced = false;
    for (int a = 0; a < areas.size(); a++) {
        ropod_ros_msgs::Area &area = areas[a];
        for (int s = 0; s < area.sub_areas.size(); s++){
            ropod_ros_msgs::SubArea &subarea = area.sub_areas[s];

            vector<int> orderedVertices = getOrderedVertices(route, a, s);

            if(!orderedVertices.empty()){
                Vector2D p1 = Vector2D(subarea.geometry.vertices[orderedVertices[0]].x, subarea.geometry.vertices[orderedVertices[0]].y);
                Vector2D p2 = Vector2D(subarea.geometry.vertices[orderedVertices[1]].x, subarea.geometry.vertices[orderedVertices[1]].y);
                Vector2D p3 = Vector2D(subarea.geometry.vertices[orderedVertices[2]].x, subarea.geometry.vertices[orderedVertices[2]].y);
                Vector2D p4 = Vector2D(subarea.geometry.vertices[orderedVertices[3]].x, subarea.geometry.vertices[orderedVertices[3]].y);

                //Build tube
                double minWidth = model.width()+extraSpace;
                double maxWidth = model.turnWidth()+extraSpace;
                double width = minWidth;
                double width1 = p1.distance(p4) > minWidth ? p1.distance(p4) : minWidth;
                width1 = width1 > maxWidth ? maxWidth : width1;
                double width2 = p2.distance(p3) > minWidth ? p2.distance(p3) : minWidth;
                width2 = width2 > maxWidth ? maxWidth : width2;

                if(area.type == "junction"){
                    width = maxWidth;
                    if(width > p2.distance(p3)){fit = false;}
                    Vector2D offsetPoint;
                    Vector2D point;
                    Vector2D point2 = ((p1-p2).unit().transform(0,0,-M_PI_2))*(width/2 + wallOffset) + p2;
                    if(!firstTubePlaced){
                        if(width > p1.distance(p4)){fit = false;}
                        tubes.emplace_back(Tube(model.pose.toVector(), width, point2, width, junctionSpeed));
                        firstTubePlaced = true;
                    }
                    else{
                        Corner corner = getJunctionDirection(route, a, s);
                        switch (corner){
                            case Corner_None:
                                addPoint(point2, width2, junctionSpeed);
                                cout << "Junction > Straight" << endl;
                                break;
                            case Corner_Left:
                                offsetPoint = ((p2-p1).unit())*(width/2 + wallOffset) + p1;
                                point = ((p1-offsetPoint).unit().transform(0,0,-M_PI_2))*(width/2 + wallOffset) + offsetPoint;
                                addPoint(point, width, junctionSpeed);
                                cout << "Junction > Left" << endl;
                                break;
                            case Corner_Right:
                                offsetPoint = ((p1-p2).unit())*(width/2 + wallOffset) + p2;
                                point = ((p2-offsetPoint).unit().transform(0,0,M_PI_2))*(width/2 + wallOffset) + offsetPoint;
                                addPoint(point, width, junctionSpeed);
                                cout << "Junction > Right" << endl;
                                break;
                        }
                    }
                }else{
                    if(width > p2.distance(p3)){fit = false;}
                    Vector2D point1 = ((p2-p1).unit().transform(0,0,M_PI_2))*(width/2 + wallOffset) + p1;
                    Vector2D point2 = ((p1-p2).unit().transform(0,0,-M_PI_2))*(width/2 + wallOffset) + p2;
                    if(!firstTubePlaced){
                        if(width > p1.distance(p4)){fit = false;}
                        //tubes.emplace_back(Tube(point1, maxWidth, point2, width, corridorSpeed));
                        tubes.emplace_back(Tube(model.pose.toVector(), maxWidth, point2, width, corridorSpeed));
                        firstTubePlaced = true;
                    }
                    else{
                        addPoint(point2, width, corridorSpeed);
                    }
                }
            }
        }
    }
    if(!tubes.empty()){
        Vector2D point = (tubes[tubes.size()-1].p2 - tubes[tubes.size()-1].p1).unit()*0.1 + tubes[tubes.size()-1].p2;
		addPoint(point, tubes[tubes.size()-1].width2, 0.3); //final point with speed 0
	}
    return fit;
}

vector<int> Tubes::getConnectedVertices(ropod_ros_msgs::RoutePlannerResult &route, int a, int s, bool &forward, bool &found){
    vector<int> connectingVertices;

    std::vector<ropod_ros_msgs::Area> &areas = route.areas;
    ropod_ros_msgs::Area &area = areas[a];
    ropod_ros_msgs::Area *area2;
    ropod_ros_msgs::SubArea &subarea = area.sub_areas[s];
    ropod_ros_msgs::SubArea *subarea2;

    found = false;
    bool subarea2found = false;

    for(int tryCounter = 0; tryCounter < 2; tryCounter++){
        if(forward && !subarea2found){
            if (s < area.sub_areas.size() - 1) { //if there is still an subarea left
                subarea2 = &area.sub_areas[s + 1];
                subarea2found = true;
            }else {
                if (a < areas.size() - 1) { //if there is still an area left
                    area2 = &areas[a + 1];
                    if (!area2->sub_areas.empty()) {
                        subarea2 = &area2->sub_areas[0];
                        subarea2found = true;
                    }
                }
            }
            if(!subarea2found){forward = false;}
        }else if(!forward && !subarea2found){ //backwards
            if (s > 0) {
                subarea2 = &area.sub_areas[s - 1];
                subarea2found = true;
            } else if (a > 0) {
                area2 = &areas[a - 1];
                if (!area2->sub_areas.empty()) {
                    subarea2 = &area2->sub_areas[area2->sub_areas.size() - 1];
                    subarea2found = true;
                }
            }
            if(!subarea2found){forward = true;}
        }
    }

    if (subarea2found) {
        int matchingVertex1 = -1, matchingVertex2 = -1;
        bool first = true;
        for (int sa1 = 0; sa1 < subarea.geometry.vertices.size(); sa1++) {
            for (int sa2 = 0; sa2 < subarea2->geometry.vertices.size(); sa2++) {
                if (subarea.geometry.vertices[sa1].id == subarea2->geometry.vertices[sa2].id) {
                    if (first) { matchingVertex1 = sa1; first = false; }
                    else { matchingVertex2 = sa1; }
                }
            }
        }
        if(matchingVertex1 != -1 && matchingVertex2 != -1){
            found = true;
            connectingVertices.emplace_back(matchingVertex1);
            connectingVertices.emplace_back(matchingVertex2);
        }
    }
    return connectingVertices;
}

vector<int> Tubes::getOrderedVertices(ropod_ros_msgs::RoutePlannerResult &route, int a, int s) {
    vector<int> orderedVetices;
    std::vector<ropod_ros_msgs::Area> &areas = route.areas;
    ropod_ros_msgs::Area &area = areas[a];
    ropod_ros_msgs::SubArea &subarea = area.sub_areas[s];

    bool connectedVerticesFound = false;
    bool forward = true;
    vector<int> connectedVertices = getConnectedVertices(route, a, s, forward, connectedVerticesFound);

    if (connectedVerticesFound) {
        int beginindex = 0;
        int matchingVertex1 = connectedVertices[0];
        int matchingVertex2 = connectedVertices[1];
        if(forward){
            if (matchingVertex2 == matchingVertex1 + 1) {
                beginindex = (4 + matchingVertex1 - 1) % 4;
            } else {
                beginindex = (4 + matchingVertex2 - 1) % 4;
            }
        }else{
            if (matchingVertex2 == matchingVertex1 + 1) {
                beginindex = matchingVertex2;
            } else {
                beginindex = matchingVertex1;
            }
        }
        orderedVetices = {(beginindex+0) % 4, (beginindex+1) % 4, (beginindex+2) % 4, (beginindex+3) % 4};
    }
    return orderedVetices;
}

Corner Tubes::getJunctionDirection(ropod_ros_msgs::RoutePlannerResult &route, int a, int s){
    Corner corner = Corner_None;

    bool connectedVerticesFound = false;
    bool forward = false;
    vector<int> connectedVertices = getConnectedVertices(route, a, s, forward, connectedVerticesFound);
    vector<int> orderedVertices = getOrderedVertices(route, a, s);
    int beginIndex = orderedVertices[0];

    connectedVertices[0] = (4+connectedVertices[0]-beginIndex)%4;
    connectedVertices[1] = (4+connectedVertices[1]-beginIndex)%4;

    if(connectedVerticesFound && !forward){
        if(connectedVertices == vector<int>{0,3} || connectedVertices == vector<int>{3,0}){ corner = Corner_None; }
        if(connectedVertices == vector<int>{2,3} || connectedVertices == vector<int>{3,2}){ corner = Corner_Left; }
        if(connectedVertices == vector<int>{0,1} || connectedVertices == vector<int>{1,0}){ corner = Corner_Right; }
    }
    return corner;
}


void Tubes::visualizePlan(ropod_ros_msgs::RoutePlannerResult &route, Visualization &canvas){
    std::vector<ropod_ros_msgs::Area> &areas = route.areas;
    for(auto &area : areas){
        for(auto &subarea : area.sub_areas){
            vector<Vector2D> points;
            for(auto &vertex : subarea.geometry.vertices){
                points.emplace_back(Vector2D(vertex.x, vertex.y));
            }
            canvas.polygon(points, Color(0,50,200), Thin);
        }
    }
}

void Tubes::visualizeRightWall(ropod_ros_msgs::RoutePlannerResult &route, Visualization &canvas) {
    std::vector<ropod_ros_msgs::Area> &areas = route.areas;
    for (int a = 0; a < areas.size(); a++) {
        ropod_ros_msgs::Area &area = areas[a];
        for (int s = 0; s < area.sub_areas.size(); s++) {
            ropod_ros_msgs::SubArea &subarea = area.sub_areas[s];
            vector<int> orderedVertices = getOrderedVertices(route, a, s);
            Vector2D p1 = Vector2D(subarea.geometry.vertices[orderedVertices[0]].x, subarea.geometry.vertices[orderedVertices[0]].y);
            Vector2D p2 = Vector2D(subarea.geometry.vertices[orderedVertices[1]].x, subarea.geometry.vertices[orderedVertices[1]].y);
            canvas.arrow(p1, p2, Color(200,50,100), Thick);
        }
    }
}

void Tubes::addPoint(Vector2D p, double width, double speed, int index){
    if(!tubes.empty()) {
        double angle = 0; //TODO more than 90 degree turns add point (or find a different way to combine the sides of the tubes)
        if(angle > M_PI_2){
            Vector2D newP = p;
            newP = newP; //TODO determine newP
            addPoint(newP, width, speed, index);
            addPoint(p, width, speed, index+1);
        }else{
            if (index < 0) { // add tube at the end [default if no index is given]
                Tube &lastTube = tubes[tubes.size() - 1];
                Tube tube(lastTube.p2, lastTube.width2, p, width, speed);
                tubes.emplace_back(tube);
                connectTubes(tubes.size() - 2);
            } else if (index >= 0 && index < tubes.size()) { // insert point
                Tube &r = tubes[index];
                r.buildTube(r.p1, r.width1, p, width, speed);
                Tube n = tubes[index + 1];
                Tube tube(p, width, n.p1, n.width1, speed);
                tubes.insert(tubes.begin() + index + 1, tube);
                if(index > 0) {connectTubes(index - 1);}
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

void Tubes::avoidObstacles(unsigned int startIndex, unsigned int index, Obstacles& obstacles, Model& model, DrivingSide side, Visualization &canvas){
    if(index < tubes.size() && !obstacles.obstacles.empty()) {
        Tube &tube = tubes[index];
        bool obstruction = false;
        for (auto &obstacle : obstacles.obstacles){
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

            for (auto &obstacle : obstacles.obstacles){
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
        Vector2D newPointLeft,newPointRight;

        double anglediff = 0.1;

        double angleleft1 = (tube1.leftSide.p2 - tube1.leftSide.p1).angle();
        double angleleft2 = (tube2.leftSide.p2 - tube2.leftSide.p1).angle();
        if(abs(angleleft1-angleleft2) > anglediff){
            newPointLeft = tube1.leftSide.lineLineIntersection(tube2.leftSide);
            if(isnan(newPointLeft.x)){newPointLeft = (tube1.leftSide.p2 + tube2.leftSide.p1)/2;}
        }else{
            newPointLeft = (tube1.leftSide.p2 + tube2.leftSide.p1)/2;
        }

        double angleright1 = (tube1.rightSide.p2 - tube1.rightSide.p1).angle();
        double angleright2 = (tube2.rightSide.p2 - tube2.rightSide.p1).angle();
        if(abs(angleright1-angleright2) > anglediff){
            newPointRight = tube1.rightSide.lineLineIntersection(tube2.rightSide);
            if(isnan(newPointRight.x)){newPointRight = (tube1.rightSide.p2 + tube2.rightSide.p1)/2;}
        }else{
            newPointRight = (tube1.rightSide.p2 + tube2.rightSide.p1)/2;
        }

        tube1.setSides(End, newPointLeft, newPointRight);
        tube2.setSides(Begin, newPointLeft, newPointRight);
    }
}

int Tubes::tubeContainingPoint(Vector2D& point, int initialSearchPoint){
    int searchvalue = 0;
    bool lowerBound = false, upperBound = false;
    while(true){
        int i = initialSearchPoint+searchvalue;
        if(i < 0) lowerBound = true;
        else if(i >= tubes.size()) upperBound = true;
        else{
            Tube &testtube = tubes[i];
            if(testtube.connectedShape.polygonContainsPoint(point)){
                return i;
            }
        }
        if(lowerBound && upperBound) break; // break if index out of bounds
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
    bool lowerBound = false, upperBound = false;
    while(true){
        int i = initialSearchPoint+searchvalue;
        if(i < 0) lowerBound = true;
        else if(i >= tubes.size()) upperBound = true;
        else{
            if(getCornerArea(i).polygonContainsPoint(point)){
                return i;
            }
        }
        if(lowerBound && upperBound) break; // break if index out of bounds
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
    if(index < tubes.size()-1) {
        Vector2D v1 = tubes[index].p2 - tubes[index].p1;
        Vector2D v2 = tubes[index+1].p2 - tubes[index+1].p1;
        v2.transformThis(0,0,-v1.angle());
        if(v2.angle() < -M_PI/32){ //right turn
            side = Corner_Right;
        }else if(v2.angle() > M_PI/32){
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
    if(index < tubes.size()-1) {
        Vector2D c = getCornerPoint(index);
        if(c.valid()) {
            Vector2D p1, p2, p3, p4;
            switch (getCornerSide(index)) {
                case Corner_Left:
                    p1 = c;
                    p2 = tubes[index].rightSide.lineProjectionPointConstrained(c);
                    p3 = tubes[index].rightSide.p2;
                    p4 = tubes[index + 1].rightSide.lineProjectionPointConstrained(c);
                    break;
                case Corner_Right:
                    p1 = c;
                    p2 = tubes[index + 1].leftSide.lineProjectionPointConstrained(c);
                    p3 = tubes[index].leftSide.p2;
                    p4 = tubes[index].leftSide.lineProjectionPointConstrained(c);
                    break;
                case Corner_None:
                    break;
            }
            cornerArea = Polygon({p1, p2, p3, p4});
        }
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
        tube.showSides(canvas, Color(255*(1-tube.velocity.length()),255*(tube.velocity.length()),100), Thin);
    }
    for(unsigned int i = 0; i < tubes.size(); i++){
        canvas.point(getCornerPoint(i), Color(255,0,0), Thick);
        canvas.polygon(getCornerArea(i).vertices, Color(100,100,200), Thin);
    }
}

void Tubes::recover(Model &model){
    //get closest tube.
    Line closestTube;
    int index = -1;
    Vector2D objectPosition = model.pose.toVector();
    for(int i = 0; i < tubes.size(); i++){
        Tube &tube = tubes[i];
        Line tubeLine = Line(tube.p1, tube.p2);
        Line dist = Line(objectPosition, tubeLine.lineProjectionPointConstrained(objectPosition));
        if(dist.length() < closestTube.length() || index == -1){
            closestTube = dist;
            index = i;
        }
    }
    if(index != -1){
        cout << "Recover > ajust tube [FIX WHEN EXISTING TUBE POINT IS TOO CLOSE]" << endl;
        vector<Vector2D> boundingBox = model.footprint.boundingBoxRotated(model.pose.a);
        Vector2D frontPoint = (boundingBox[1]+boundingBox[2])/2;
        Vector2D backPoint = (boundingBox[0]+boundingBox[3])/2;
        addPoint(backPoint, model.turnWidth()+0.4, tubes[index].velocity.length(), index);
        addPoint(frontPoint, model.turnWidth()+0.4, tubes[index].velocity.length(), index+1);

    }
    //inset new tube.
}


