//
// Created by bob on 25-09-19.
//

#include "Visualization.h"

Visualization::Visualization(){}

void Visualization::point(const Vector2D &p, const Color &c, unsigned int thickness){}

void Visualization::line(const Vector2D &p1, const Vector2D &p2, const Color &c, unsigned int thickness) {}

void Visualization::arrow(const Vector2D &p1, const Vector2D &p2, const Color &c, unsigned int thickness) {}

void Visualization::circle(const Vector2D &p, const double radius, const Color &c, int drawstyle) {}

void Visualization::rectangle(const Vector2D &p1, const Vector2D &p2, const Color &c, int drawstyle) {}

void Visualization::polygon(const vector<Vector2D> &points, const Color &c, int drawstyle) {}
