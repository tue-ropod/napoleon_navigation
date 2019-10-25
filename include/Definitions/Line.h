//
// Created by bob on 27-09-19.
//

#ifndef NAVIGATION_LINE_H
#define NAVIGATION_LINE_H

#include <Visualization/Visualization.h>
#include <iostream>
#include "Vector2D.h"
#include "Circle.h"

class Line {
public:
    Vector2D p1;
    Vector2D p2;

    Line() = default;

    Line(Vector2D p1_, Vector2D p2_){
        p1 = p1_;
        p2 = p2_;
    }

    double length(){
        return (p1-p2).length();
    }

    bool lineLineCollision(Line other){
        double x1 = this->p1.x;
        double y1 = this->p1.y;
        double x2 = this->p2.x;
        double y2 = this->p2.y;

        double x3 = other.p1.x;
        double y3 = other.p1.y;
        double x4 = other.p2.x;
        double y4 = other.p2.y;

        double denominator = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);
        double t = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/denominator;
        double u = -((x1-x2)*(y1-y3)-(y1-y2)*(x1-x3))/denominator;

        bool collision = t >= 0 && t <= 1 && u >=0 && u <= 1;

        return  collision;
    }

    Vector2D lineLineIntersection(Line other){
        double x1 = this->p1.x;
        double y1 = this->p1.y;
        double x2 = this->p2.x;
        double y2 = this->p2.y;

        double x3 = other.p1.x;
        double y3 = other.p1.y;
        double x4 = other.p2.x;
        double y4 = other.p2.y;

        double denominator = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);
        double px, py;
        if(abs(denominator) > 0.01 && !isnan(abs(denominator)) && !isinf(abs(denominator))) {
            px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator;
            py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator;
        }else{
            px = NAN;
            py = NAN;
        }

        return {px, py};
    }

    bool lineInCircle(Circle& c){
        return lineDistanceToPoint(c.point) < c.radius;
    }

    double lineDistanceToPoint(Vector2D& p){
        Vector2D d = p - lineProjectionPointConstrained(p);
        return d.length();
    }

    double lineProjectionDistanceToPoint(Vector2D& p){
        Vector2D d = p - lineProjectionPoint(p);
        return d.length();
    }

    Vector2D lineProjectionPoint(Vector2D& p){
        Vector2D a = p - p1;
        Vector2D b = p2 - p1;
        return b.unit() * (a.dot(b.unit())) + p1;
    }

    Vector2D lineProjectionPointConstrained(Vector2D& p){
        Vector2D a = p - p1;
        Vector2D b = p2 - p1;
        double length = a.dot(b.unit());
        length = length < 0 ? 0 : length;
        length = length > b.length() ? b.length() : length;
        return b.unit() * length + p1;
    }

    Line shortestLineTo(Line& l){
        Line connectionLine;
        Vector2D intersectionPoint;
        if(lineLineCollision(l)){
            Vector2D point = lineLineIntersection(l);
            connectionLine.p1 = point;
            connectionLine.p2 = point;
        }else{
            Vector2D point1 = p1;
            Vector2D point2 = l.p1;
            double mindist = -1;
            for(int i = 0; i < 4; i++){
                Vector2D testPoint1, testPoint2;
                switch(i){
                    case 0:
                        testPoint1 = p1;
                        testPoint2 = l.lineProjectionPointConstrained(testPoint1);
                        break;
                    case 1:
                        testPoint1 = p2;
                        testPoint2 = l.lineProjectionPointConstrained(testPoint1);
                        break;
                    case 2:
                        testPoint2 = l.p1;
                        testPoint1 = lineProjectionPointConstrained(testPoint2);
                        break;
                    case 3:
                        testPoint2 = l.p2;
                        testPoint1 = lineProjectionPointConstrained(testPoint2);
                        break;
                }
                double testdist = testPoint1.distance(testPoint2);
                if(testdist < mindist || mindist == -1){
                    mindist = testdist;
                    point1 = testPoint1;
                    point2 = testPoint2;
                }
            }
            connectionLine.p1 = point1;
            connectionLine.p2 = point2;
        }
        return connectionLine;
    }
};

inline bool linesort(Line& l1, Line& l2){
    return (l1.length() < l2.length());
}

#endif //NAVIGATION_LINE_H
