#ifndef NAP_GEOM_H
#define NAP_GEOM_H

using namespace std;
#include <string>
#include <vector>

class Point{
public:
    double x, y;
    Point();
    Point(double xval, double yval);
    Point sub(Point &b);
    Point add(Point &b);
};

class PointID : public Point {
public:
    string id;
    PointID();
    PointID(double xval, double yval, string id);
};

class AreaQuad {
public:
    Point p0, p1, p2, p3;
    AreaQuad();
    AreaQuad(Point p0val, Point p1val, Point p2val, Point p3val);
    bool contains(Point &q);
};

class AreaQuadID : public AreaQuad{
public:
    PointID p0, p1, p2, p3;
    int id;
    string type;
    AreaQuadID();
    AreaQuadID(PointID p0val, PointID p1val, PointID p2val, PointID p3val, int idval, string typeval);
    vector<string> getPointIDs();
    Point center();
};


#endif
