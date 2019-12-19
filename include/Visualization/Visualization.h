//
// Created by bob on 25-09-19.
//

#ifndef NAVIGATION_VISUALIZATION_H
#define NAVIGATION_VISUALIZATION_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <string>
#include <utility>
#include <Definitions/Pose2D.h>

using namespace std;

enum DrawStyle {Filled = -1, Thin = 2, Thick = 4};

class Color{
public:
    unsigned int red;
    unsigned int green;
    unsigned int blue;
    unsigned int alpha;
    Color() = default;
    Color(unsigned int red_, unsigned int green_, unsigned int blue_, unsigned int alpha_ = 255){
        red = red_;
        green = green_;
        blue = blue_;
        alpha = alpha_;
    }
};

class Visualization{
protected:
    vector<int> idCounters = {0};
    vector<string> idNames = {"unassigned"};

public:
    string idName = "unassigned";

    Visualization();

    int getId(string name);
    void resetId();

    virtual void point(const Vector2D& p, const Color& c, unsigned int thickness = Thin);
    virtual void line(const Vector2D& p1, const Vector2D& p2, const Color& c, unsigned int thickness = Thin);
    virtual void arrow(const Vector2D& p1, const Vector2D& p2, const Color& c, unsigned int thickness = Thin);
    virtual void circle(const Vector2D& p, double radius, const Color& c, int drawstyle = Thin);
    virtual void rectangle(const Vector2D &p1, const Vector2D &p2, const Color &c, int drawstyle = Thin);
    virtual void polygon(const vector<Vector2D>& points, const Color& c, int drawstyle = Filled);
    virtual void lines(const vector<Vector2D>& points, const Color& c, int drawstyle = Thin);

};

#endif //NAVIGATION_VISUALIZATION_H
