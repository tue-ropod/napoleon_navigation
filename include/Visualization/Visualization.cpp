//
// Created by bob on 25-09-19.
//

#include "Visualization.h"

Visualization::Visualization(){}

int Visualization::getId(string name) {
    int i = -1;
    for(int n = 0; n < idNames.size(); n++){
        if(name == idNames[n]){
            i = n;
            break;
        }
    }
    if(i == -1){
        idNames.emplace_back(name);
        idCounters.emplace_back(0);
        i = idNames.size()-1;
    }
    int &idCounter = idCounters[i];
    int idtemp = idCounter;
    idCounter++;
    return idtemp;

}

void Visualization::resetId(){
    for(int &id : idCounters) {
        id = 0;
    }
}

void Visualization::point(const Vector2D &p, const Color &c, unsigned int thickness){}

void Visualization::line(const Vector2D &p1, const Vector2D &p2, const Color &c, unsigned int thickness) {}

void Visualization::arrow(const Vector2D &p1, const Vector2D &p2, const Color &c, unsigned int thickness) {}

void Visualization::circle(const Vector2D &p, const double radius, const Color &c, int drawstyle) {}

void Visualization::rectangle(const Vector2D &p1, const Vector2D &p2, const Color &c, int drawstyle) {}

void Visualization::polygon(const vector<Vector2D> &points, const Color &c, int drawstyle) {}

void Visualization::lines(const vector<Vector2D> &points, const Color &c, int drawstyle) {}