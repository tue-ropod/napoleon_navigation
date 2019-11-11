//
// Created by bob on 02-10-19.
//

#ifndef NAVIGATION_TUBES_H
#define NAVIGATION_TUBES_H

#include <Obstacles/Obstacles.h>
#include <Model/Model.h>
#include "Tube.h"
#include "iostream"
#include <ropod_ros_msgs/RoutePlannerAction.h>
#include <Communication/Communication.h>


class Model;

enum DrivingSide {DrivingSide_Left, DrivingSide_Right, DrivingSide_Automatic};
enum Corner {Corner_Left, Corner_Right, Corner_None};

class Tubes {
public:
    vector<Tube> tubes;

    Tubes() = default;
    Tubes(const Tube& tube);
    void addPoint(Vector2D p, double width, double speed, int index = -1);
    void removePoint(unsigned int index);
    void connectTubes(unsigned int index);

    void recover(Model &model);
    void avoidObstacles(unsigned int startIndex, unsigned int index, Obstacles& obstacles, Model& model, DrivingSide side, Visualization& canvas);

    int tubeContainingPoint(Vector2D& point, int initialSearchPoint = 0);
    int tubeCornerContainingPoint(Vector2D& point, int initialSearchPoint = 0);
    Corner getCornerSide(unsigned int index);
    Vector2D getCornerPoint(unsigned int index);
    Polygon getCornerArea(unsigned int index);

    bool convertRoute(Communication &comm, Model &model, Visualization &canvas);

    void showOriginalTubes(Visualization& canvas);
    void showTubes(Visualization& canvas);
    void showSides(Visualization& canvas);
    void visualizePlan(ropod_ros_msgs::RoutePlannerResult &route, Visualization &canvas);
    void visualizeRightWall(ropod_ros_msgs::RoutePlannerResult &route, Visualization &canvas);

private:
    vector<int> getConnectedVertices(ropod_ros_msgs::RoutePlannerResult &route, int a, int s, bool &forward, bool &found);
    vector<int> getOrderedVertices(ropod_ros_msgs::RoutePlannerResult &route, int a, int s);
    Corner getJunctionDirection(ropod_ros_msgs::RoutePlannerResult &route, int a, int s);

};


#endif //NAVIGATION_TUBES_H

