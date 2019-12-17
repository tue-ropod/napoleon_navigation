//
// Created by bob on 24-10-19.
//

#include "Obstacles.h"

void Obstacles::show(Visualization &canvas, Color c, int drawStyle) {
    canvas.idName = "Obstacles";
    for(auto &obstacle : obstacles){
        obstacle.show(canvas, c, drawStyle);
    }
}

void Obstacles::update(double dt) {
    for(auto &obstacle : obstacles){
        obstacle.update(dt);
    }
}

void Obstacles::removeOldVisibleObstacles(Polygon &area) {
    for(int o = 0; o < obstacles.size(); o++){
        Obstacle &obs = obstacles[o];
        if(obs.lifeTime < 0 || (obs.lifeTime > 2 && area.polygonPolygonCollision(obs.footprint))){
            obstacles.erase(obstacles.begin()+o);
            o--;
        }
    }
}

void Obstacles::removeOldSelectiveObstacles(){
    double mergeDistance = 0.2;
    double mergeAngle = M_PI/4;
    for(int o1 = 0; o1 < obstacles.size()-1; o1++){ //one way check > [0-1 | 0-2 | 0-3 | 1-2 | 1-3 | 2-3]
        Obstacle &obs1 = obstacles[o1];
        Line l1 = Line(obs1.footprint.vertices[0], obs1.footprint.vertices[1]);
        for(int o2 = o1+1; o2 < obstacles.size(); o2++){
            Obstacle &obs2 = obstacles[o2];
            Line l2 = Line(obs2.footprint.vertices[0], obs2.footprint.vertices[1]);
            if(obs1.lifeTime > 2 && obs2.lifeTime > 2){
                double anglediff = l1.angle() - l2.angle();
                smallestAngle(anglediff);
                if (l1.shortestLineTo(l2).length() < mergeDistance && anglediff < mergeAngle) {
                    if(l1.length() < l2.length()){
                        obs1.lifeTime = -1;
                    }else{
                        obs2.lifeTime = -1;
                    }
                }
            }
        }
    }
    for(int o = 0; o < obstacles.size(); o++){
        Obstacle &obs = obstacles[o];
        if(obs.lifeTime < 0 || obs.lifeTime > 60 || (obs.lifeTime > 2 && obs.weight < 30)){
            obstacles.erase(obstacles.begin()+o);
            o--;
        }
    }
}
