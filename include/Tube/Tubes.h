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
    void avoidObstacles(unsigned int startIndex, unsigned int index, Obstacles& obstacles, Model& model, DrivingSide side, Visualization& canvas);
    void connectTubes(unsigned int index);
    void convertRoute(ropod_ros_msgs::RoutePlannerResult &route, Model &model, Visualization &canvas);
    int tubeContainingPoint(Vector2D& point, int initialSearchPoint = 0);
    int tubeCornerContainingPoint(Vector2D& point, int initialSearchPoint = 0);
    Corner getCornerSide(unsigned int index);
    Vector2D getCornerPoint(unsigned int index);
    Polygon getCornerArea(unsigned int index);
    void showOriginalTubes(Visualization& canvas);
    void showTubes(Visualization& canvas);
    void showSides(Visualization& canvas);

};


#endif //NAVIGATION_TUBES_H

