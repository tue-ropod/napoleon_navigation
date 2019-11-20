//
// Created by bob on 02-10-19.
//

#ifndef NAVIGATION_TUBE_H
#define NAVIGATION_TUBE_H

#include <Definitions/Vector2D.h>
#include <Definitions/Polygon.h>
#include <utility>

enum Place {Begin, End};

class Tube {
public:
    Vector2D p1, p2;
    double width1, width2;
    Vector2D velocity;
    Polygon shape, connectedShape;
    Line leftSide, rightSide;

    Tube() = default;
    //Tube(Polygon shape_, unsigned int alignpoint1, unsigned int alignpoint2, double speed);
    Tube(const Vector2D& p1_, double width1_, const Vector2D& p2_, double width2_, double speed);
    Tube(const Vector2D& p1Left, const Vector2D& p1Right, const Vector2D& p2Left, const Vector2D& p2Right, double speed);
    void buildTube(Vector2D p1_, double width1_, Vector2D p2_, double width2, double speed);
    void buildTube(Vector2D p1Left, Vector2D p1Right, Vector2D p2Left, Vector2D p2Right, double speed);
    void resetSides(Place place);
    void setSides(Place place, Vector2D leftPoint, Vector2D rightPoint);
    void showOriginalTube(Visualization& canvas, Color c, int drawstyle = Thin);
    void showTube(Visualization& canvas, Color c, int drawstyle = Thin);
    void showSides(Visualization& canvas, Color c, int drawstyle = Thin);
};


#endif //NAVIGATION_TUBE_H
