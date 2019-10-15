//
// Created by bob on 02-10-19.
//

#ifndef NAVIGATION_TUBES_H
#define NAVIGATION_TUBES_H

#include <Obstacles/Obstacle.h>
#include <Model/Model.h>
#include "Tube.h"
#include "iostream"

class Model;

enum DrivingSide {DrivingSide_Left, DrivingSide_Right, DrivingSide_Automatic};
enum Corner {Corner_Left, Corner_Right, Corner_None};

class Tubes {
public:
    vector<Tube> tubes;

    Tubes(const Tube& tube);
    void addPoint(Vector2D p, double width, double speed, int index = -1);
    void removePoint(unsigned int index);
    void avoidObstacles(unsigned int startIndex, unsigned int index, vector<Obstacle>& obstacles, Model& model, DrivingSide side, Visualization& canvas);
    void connectTubes(unsigned int index);
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

