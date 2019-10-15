//
// Created by bob on 25-09-19.
//

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <string>
#include <Definitions/Pose2D.h>

#ifndef NAVIGATION_VISUALIZATION_H
#define NAVIGATION_VISUALIZATION_H

using namespace std;

class Color{
public:
    unsigned int red;
    unsigned int green;
    unsigned int blue;
    unsigned int alpha;
    Color(unsigned int red_, unsigned int green_, unsigned int blue_, unsigned int alpha_ = 255){
        red = red_;
        green = green_;
        blue = blue_;
        alpha = alpha_;
    }
};

enum DrawStyle {Filled = -1, Thin = 1, Thick = 3};

class Visualization{
private:
    double pixelsPerMeter;
    cv::Mat canvas;
    int width;
    int height;
    Pose2D origin = Pose2D(0,0,0);

public:
    Visualization(int width_, int height_, int pixelsPerMeter_);
    void setorigin(const Pose2D& origin_);
    Pose2D getWindowMidOffset();
    void emptycanvas();
    void visualize();
    cv::Point worldToCanvas(const Vector2D& p);
    void point(const Vector2D& p, const Color& c, unsigned int thickness = Thin);
    void line(const Vector2D& p1, const Vector2D& p2, const Color& c, unsigned int thickness = Thin);
    void arrow(const Vector2D& p1, const Vector2D& p2, const Color& c, unsigned int thickness = Thin);
    void circle(const Vector2D& p, double radius, const Color& c, int drawstyle = Thin);
    void rectangle(const Vector2D &p1, const Vector2D &p2, const Color &c, int drawstyle = Thin);
    void polygon(const vector<Vector2D>& points, const Color& c, int drawstyle = Filled);

//    void save(const char filename[50]);
};

#endif //NAVIGATION_VISUALIZATION_H
