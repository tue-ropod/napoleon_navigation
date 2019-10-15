//
// Created by bob on 25-09-19.
//

#ifndef NAVIGATION_POLYGON_H
#define NAVIGATION_POLYGON_H

#include <utility>

#include "vector"
#include "Vector2D.h"
#include "Pose2D.h"
#include "Line.h"
#include "Circle.h"

enum Type {Open = 0, Closed = 1};

class Polygon{
public:
    vector<Vector2D> vertices;
    vector<Line> sides;
    Pose2D middle;
    bool closed;

    Polygon() = default;

    Polygon(vector<Vector2D> vertices_, Type type = Closed, bool manualmiddle = false, Pose2D middle_ = Pose2D()){
        vertices = std::move(vertices_);
        if(manualmiddle) {
            middle = middle_;
        }else{
            Vector2D m;
            for(auto & vertex : vertices){
                m = m + vertex;
            }
            m = m/vertices.size();
            middle = Pose2D(m, 0);
        }
        closed = bool(type);
        sides = getSides();
    }

    vector<Line> getSides(){
        vector<Line> s;
        int type = int(!closed); // closed > 0 open > 1
        for(int i = 0; i < vertices.size()-type; i++){
            Line side(vertices[i], vertices[(i+1)%vertices.size()]);
            s.emplace_back(side);
        }
        return s;
    }

    void scale(double x, double y){
        middle.scaleThis(x, y);
        for(auto & vertex : vertices){
            vertex.scaleThis(x, y);
        }
        sides = getSides();
    }

    void dilate(double offset){
        for(auto & side : sides){
            Vector2D normal = (side.p2 - side.p1).transform(0,0,-M_PI_2).unit();
            side.p1 = side.p1 + normal*offset;
            side.p2 = side.p2 + normal*offset;
        }
        int type = int(!closed); // closed > 0 open > 1
        for(int i = 0; i < sides.size()-type; i++){
            Line side1 = sides[i];
            Line side2 = sides[(i+1)%sides.size()];
            vertices[(i+1)%sides.size()] = side2.lineProjectionPoint(side1.p2);
        }
        sides = getSides();
    }

    void transform(double x, double y, double a, bool updateSides = true){
        middle.transformThis(x, y, 0);
        middle.rotateOrientation(a);
        for(auto & vertex : vertices){
            vertex.transformThis(x, y, 0);
            vertex.transformThis(0, 0, a);
        }
        if(updateSides) {
            sides = getSides();
        }
    }

    void rotateAroundMiddle(double a, bool updateSides = true){
        middle.rotateOrientation(a);
        for(auto & vertex : vertices){
            vertex = vertex - middle;
            vertex.transformThis(0, 0, a);
            vertex = vertex + middle;
        }
        if(updateSides) {
            sides = getSides();
        }
    }

    void transformto(Pose2D p){
        transform(-middle.x, -middle.y, 0, false);
        rotateAroundMiddle(-middle.a, false);
        transform(p.x, p.y, 0, false);
        rotateAroundMiddle(p.a, false);
        sides = getSides();
    }

    vector<Vector2D> boundingBoxRotated(double angle){
        rotateAroundMiddle(-angle, false);
        vector<Vector2D> points = boundingBox();
        rotateAroundMiddle(angle, false);
        for(auto &point : points){
            point = point - middle;
            point.transformThis(0, 0, angle);
            point = point + middle;
        }
        return points;
    }

    vector<Vector2D> boundingBox(){
        Vector2D bBmin(0,0), bBmax(0,0);
        bool init = false;
        for(auto & vertex : vertices){
            if(!init){
                bBmin.x = vertex.x;
                bBmin.y = vertex.y;
                bBmax.x = vertex.x;
                bBmax.y = vertex.y;
                init = true;
            }else {
                if (vertex.x < bBmin.x) { bBmin.x = vertex.x; }
                if (vertex.y < bBmin.y) { bBmin.y = vertex.y; }
                if (vertex.x > bBmax.x) { bBmax.x = vertex.x; }
                if (vertex.y > bBmax.y) { bBmax.y = vertex.y; }
            }
        }
        return {bBmin, Vector2D(bBmax.x, bBmin.y), bBmax, Vector2D(bBmin.x, bBmax.y)};
    }

    bool polygonContainsPoint(Vector2D& point){
        bool inside = false;
        if(closed){
            vector<Vector2D> bB = boundingBox();
            Vector2D min = bB[0], max = bB[2];
            if(point.x > min.x && point.x < max.x && point.y > min.y && point.y < max.y) {
                int count = 0;
                Line ray(Vector2D(min.x-1,min.y), point);
                for(auto & side : sides){
                    if(side.lineLineCollision(ray)){count++;}
                }
                inside = bool(count%2);
            }
        }
        return inside;
    }

    bool polygonLineCollision(Line& l){
        for(auto & side : sides){
            if(side.lineLineCollision(l)){return true;}
        }
        return false;
    }

    bool polygonPolygonCollision(Polygon& other){
        for(auto & side : sides){
            if(other.polygonLineCollision(side)){return true;}
        }
        return false;
    }

    vector<Line> sidesPolygonPolygonCollision(Polygon& other){
        vector<Line> collisionSides;
        for(auto & side : sides){
            if(other.polygonLineCollision(side)){collisionSides.emplace_back(side);}
        }
        return collisionSides;
    }

    bool polygonInCircle(Circle& c){
        for(auto & side : sides){
            if(side.lineInCircle(c)){return true;}
        }
        return false;
    }

    vector<Vector2D> projectPolygonOnLine(Line& line){
        Vector2D minProjection(0,0), maxProjection(0,0);
        double lineAngle = (line.p2-line.p1).angle();
        transform(-line.p1.x, -line.p1.y, 0, false);
        transform(0, 0, -lineAngle, false);
        vector<Vector2D> bB = boundingBox();
        minProjection = bB[0];
        maxProjection = bB[2];
        transform(0, 0, lineAngle, false);
        transform(line.p1.x, line.p1.y, 0, false);

        minProjection.y = 0;
        minProjection.transformThis(0, 0, lineAngle);
        minProjection.transformThis(line.p1.x, line.p1.y, 0);

        maxProjection.y = 0;
        maxProjection.transformThis(0, 0, lineAngle);
        maxProjection.transformThis(line.p1.x, line.p1.y, 0);

        return vector<Vector2D> {minProjection, maxProjection};
    }

    vector<double> distancePolygonToLineMinMax(Line& line){
        Vector2D minProjection(0,0), maxProjection(0,0);
        double lineAngle = (line.p2-line.p1).angle();
        transform(-line.p1.x, -line.p1.y, 0, false);
        transform(0, 0, -lineAngle, false);
        vector<Vector2D> bB = boundingBox();
        minProjection = bB[0];
        maxProjection = bB[2];
        transform(0, 0, lineAngle, false);
        transform(line.p1.x, line.p1.y, 0, false);
        double min = abs(maxProjection.y) < abs(minProjection.y) ? (maxProjection.y) : (minProjection.y);
        double max = abs(maxProjection.y) > abs(minProjection.y) ? (maxProjection.y) : (minProjection.y);
        vector<double> distances = {min, max};
        return distances;
    }
};

#endif //NAVIGATION_POLYGON_H
