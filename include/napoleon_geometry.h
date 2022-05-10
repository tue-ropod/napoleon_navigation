#ifndef NAP_GEOM_H
#define NAP_GEOM_H

using namespace std;
#include <string>
#include <vector>

class PointID;
class Point {
public:
    double x, y;
    Point();
    Point(double xval, double yval);
    Point sub(Point b);
    Point add(Point b);
    Point sub(PointID b);
    Point add(PointID b);
};

class PointID {
public:
    double x, y;
    string id;
    PointID();
    PointID(double xval, double yval, string id);
    PointID sub(Point b);
    PointID add(Point b);
    PointID sub(PointID b);
    PointID add(PointID b);
};

class AreaQuad {
public:
    Point p0, p1, p2, p3;
    AreaQuad();
    AreaQuad(Point p0val, Point p1val, Point p2val, Point p3val);
    bool contains(Point q);
};


class Rectangle {
public:
    double width;
    double depth;
    double x;
    double y;
};


class AreaQuadID {
public:
    PointID p0, p1, p2, p3;
    int id;
    string type;
    AreaQuadID();
    AreaQuadID(PointID p0val, PointID p1val, PointID p2val, PointID p3val, int idval, string typeval);
    vector<string> getPointIDs();
    bool contains(Point q);
    Point center();
};


#endif
