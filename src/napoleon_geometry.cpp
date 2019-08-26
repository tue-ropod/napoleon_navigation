#include "napoleon_geometry.h"


using namespace std;


// For all points except map points
Point::Point() {
    x = 0.0;
    y = 0.0;
}

Point::Point(double xval , double yval ) {
    x = xval;
    y = yval;
}

// Code can be cleaner if PointID inherits Point
Point Point::sub(Point b) {
    return Point(x - b.x, y - b.y);
}

Point Point::add(Point b) {
    return Point(x + b.x, y + b.y);
}

Point Point::sub(PointID b) {
    return Point(x - b.x, y - b.y);
}

Point Point::add(PointID b) {
    return Point(x + b.x, y + b.y);
}

// For map points
PointID::PointID() {
	x = 0.0;
	y = 0.0;
	id = " ";
}
PointID::PointID(double xval, double yval, string idval) {
    x = xval;
    y = yval;
    id = idval;
}

PointID PointID::sub(Point b) {
    return PointID(x - b.x, y - b.y, id);
}

PointID PointID::add(Point b) {
    return PointID(x + b.x, y + b.y, id);
}

PointID PointID::sub(PointID b) {
    return PointID(x - b.x, y - b.y, id);
}

PointID PointID::add(PointID b) {
    return PointID(x + b.x, y + b.y, id);
}


// For dynamic areas like entries
AreaQuad::AreaQuad() {
}
AreaQuad::AreaQuad(Point p0val, Point p1val, Point p2val, Point p3val) {
    p0 = p0val;
    p1 = p1val;
    p2 = p2val;
    p3 = p3val;
}

// empty constructor
AreaQuadID::AreaQuadID() {
    type = "none";
}

// For static areas like hallways and intersections
AreaQuadID::AreaQuadID(PointID p0val, PointID p1val, PointID p2val, PointID p3val, int idval, string typeval) {
    p0 = p0val;
    p1 = p1val;
    p2 = p2val;
    p3 = p3val;
    id = idval;
    type = typeval;
}



bool AreaQuad::contains(Point q) {
    bool c = false;
    int i, j = 0;
    int nvert = 4;
    double vertx[4] = {p0.x, p1.x, p2.x, p3.x};
    double verty[4] = {p0.y, p1.y, p2.y, p3.y};
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if ( ((verty[i]>q.y) != (verty[j]>q.y)) && (q.x < (vertx[j]-vertx[i]) * (q.y-verty[i]) / (verty[j]-verty[i]) + vertx[i]) ){
            c = !c;
        }
    }
    return c;
}

bool AreaQuadID::contains(Point q) {
    bool c = false;
    int i, j = 0;
    int nvert = 4;
    double vertx[4] = {p0.x, p1.x, p2.x, p3.x};
    double verty[4] = {p0.y, p1.y, p2.y, p3.y};
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
        if ( ((verty[i]>q.y) != (verty[j]>q.y)) && (q.x < (vertx[j]-vertx[i]) * (q.y-verty[i]) / (verty[j]-verty[i]) + vertx[i]) ){
            c = !c;
        }
    }
    return c;
}

Point AreaQuadID::center() {
    Point a((p0.x+p1.x+p2.x+p3.x)/4, (p0.y+p1.y+p2.y+p3.y)/4);
    return a;
}

vector<string> AreaQuadID::getPointIDs() {
    vector<string> s {p0.id, p1.id, p2.id, p3.id};
    return s;
}

