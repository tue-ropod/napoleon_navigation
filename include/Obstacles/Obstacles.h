//
// Created by bob on 24-10-19.
//

#ifndef SRC_OBSTACLES_H
#define SRC_OBSTACLES_H

#include <Obstacles/Obstacle.h>

class Visualization;

class Obstacles {
public:
    vector<Obstacle> obstacles;
    void show(Visualization &canvas, Color c, int drawStyle);
    void update(double dt);
    void removeOldVisibleObstacles(Polygon &area);
    void removeOldSelectiveObstacles();
};

#endif //SRC_OBSTACLES_H
