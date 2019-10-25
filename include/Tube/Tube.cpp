//
// Created by bob on 02-10-19.
//

#include "Tube.h"

//Tube::Tube(Polygon shape_, unsigned int alignpoint1, unsigned int alignpoint2, double speed){
//    shape = std::move(shape_);
//    int nVertices = shape.vertices.size();
//    Vector2D p;
//    if(alignpoint1 < nVertices && alignpoint2 < nVertices && alignpoint1 != alignpoint2){
//        p = shape.vertices[alignpoint2] - shape.vertices[alignpoint1];
//        p = p.unitThis();
//    }
//    velocity = p*speed;
//}

Tube::Tube(Vector2D p1_, double width1_, Vector2D p2_, double width2_, double speed){
    p1 = p1_;
    width1 = width1_;
    p2 = p2_;
    width2=  width2_;
    buildTube(p1, width1, p2, width2, speed);
}

void Tube::buildTube(Vector2D p1_, double width1_, Vector2D p2_, double width2_, double speed){
    p1 = p1_;
    width1 = width1_;
    p2 = p2_;
    width2 = width2_;

    speed = speed > 1 ? 1 : speed;
    speed = speed < 0 ? 0 : speed;

    Vector2D v1, v2, v3, v4;
    Vector2D dir = (p2 - p1).unit();
    dir = dir.length() == 0 ? Vector2D(1,0) : dir;
    velocity = dir*speed;

    double r1 = width1/2, r2 = width2/2;
    double alpha = atan(abs(r1-r2)/p1.distance(p2));
    if(r1 < r2){alpha *= -1;}
    double d = tan(alpha)*r1;
    double l = sqrt((r1*r1) + ((r1+d)*(r1+d)));
    double beta = atan(r1 / (r1+d));

    dir.transformThis(0, 0, M_PI_2 + beta);
    v1 = p1 + dir*l;
    dir.transformThis(0, 0, M_PI - 2*beta);
    v2 = p1 + dir*l;

    alpha = atan(abs(r1-r2)/p1.distance(p2));
    if(r2 < r1){alpha *= -1;}
    d = tan(alpha)*r2;
    l = sqrt((r2*r2) + ((r2+d)*(r2+d)));
    beta = atan(r2 / (r2+d));

    dir = (p1 - p2).unit();
    dir = dir.length() == 0 ? Vector2D(-1,0) : dir;
    dir.transformThis(0, 0, M_PI_2 + beta);
    v3 = p2 + dir*l;
    dir.transformThis(0, 0, M_PI - 2*beta);
    v4 = p2 + dir*l;

//    dir.transformThis(0, 0, M_PI_2 + M_PI_4);
//    v1 = p1 + dir * (width1 / 2) * sqrt(2);
//    dir.transformThis(0, 0, M_PI_2);
//    v2 = p1 + dir * (width1 / 2) * sqrt(2);
//
//    dir = (p1 - p2).unit();
//    dir = dir.length() == 0 ? Vector2D(-1,0) : dir;
//    dir.transformThis(0, 0, M_PI_2 + M_PI_4);
//    v3 = p2 + dir * (width2 / 2) * sqrt(2);
//    dir.transformThis(0, 0, M_PI_2);
//    v4 = p2 + dir * (width2 / 2) * sqrt(2);

    shape = Polygon({v1,v2,v3,v4});
    connectedShape = Polygon({v1,v2,v3,v4});
    leftSide = Line(v1, v4);
    rightSide = Line(v2, v3);
}

void Tube::resetSides(Place place){
    switch(place){
        case Begin:
            leftSide = Line(shape.vertices[0], leftSide.p2);
            rightSide = Line(shape.vertices[1], rightSide.p2);
            connectedShape.vertices[0] = leftSide.p1;
            connectedShape.vertices[1] = rightSide.p1;
            break;
        case End:
            leftSide = Line(leftSide.p1, shape.vertices[3]);
            rightSide = Line(rightSide.p1, shape.vertices[2]);
            connectedShape.vertices[3] = leftSide.p2;
            connectedShape.vertices[2] = rightSide.p2;
            break;
    }
}

void Tube::setSides(Place place, Vector2D leftPoint, Vector2D rightPoint){
    switch(place){
        case Begin:
            leftSide = Line(leftPoint, leftSide.p2);
            rightSide = Line(rightPoint, rightSide.p2);
            connectedShape.vertices[0] = leftSide.p1;
            connectedShape.vertices[1] = rightSide.p1;
            break;
        case End:
            leftSide = Line(leftSide.p1, leftPoint);
            rightSide = Line(rightSide.p1, rightPoint);
            connectedShape.vertices[3] = leftSide.p2;
            connectedShape.vertices[2] = rightSide.p2;
            break;
    }
}

void Tube::showOriginalTube(Visualization& canvas, Color c, int drawstyle){
    canvas.polygon(shape.vertices, c, drawstyle);
    canvas.circle(p1, width1/2, c, Thin);
    canvas.circle(p2, width2/2, c, Thin);
}

void Tube::showTube(Visualization& canvas, Color c, int drawstyle) {
    canvas.polygon(connectedShape.vertices, c, drawstyle);
}

void Tube::showSides(Visualization& canvas, Color c, int drawstyle) {
    canvas.line(leftSide.p1, leftSide.p2, c, drawstyle);
    canvas.line(rightSide.p1, rightSide.p2, c, drawstyle);
}
