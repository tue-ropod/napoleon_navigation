//
// Created by bob on 24-10-19.
//

#ifndef SRC_OBSTACLES_H
#define SRC_OBSTACLES_H

#include <Obstacles/Obstacle.h>
#include <Visualization/Visualization.h>

class Obstacles {
public:
    vector<Obstacle> obstacles;
    void show(Visualization &canvas, Color c, int drawStyle);
};

#endif //SRC_OBSTACLES_H
