//
// Created by bob on 17-10-19.
//

#ifndef SRC_VISUALIZATIONOPENCV_H
#define SRC_VISUALIZATIONOPENCV_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <string>
#include <Definitions/Pose2D.h>
#include <Visualization/Visualization.h>

using namespace std;

class VisualizationOpenCV : public Visualization{
private:
    double pixelsPerMeter;
    cv::Mat canvas;
    int width;
    int height;
    Pose2D origin = Pose2D(0,0,0);

public:
    VisualizationOpenCV(int width_, int height_, int pixelsPerMeter_);
    void setorigin(const Pose2D& origin_);
    Pose2D getWindowMidOffset();
    void emptycanvas();
    void visualize();
    cv::Point worldToCanvas(const Vector2D& p);
    void point(const Vector2D& p, const Color& c, unsigned int thickness = Thin) override;
    void line(const Vector2D& p1, const Vector2D& p2, const Color& c, unsigned int thickness = Thin) override;
    void arrow(const Vector2D& p1, const Vector2D& p2, const Color& c, unsigned int thickness = Thin) override;
    void circle(const Vector2D& p, double radius, const Color& c, int drawstyle = Thin) override;
    void rectangle(const Vector2D &p1, const Vector2D &p2, const Color &c, int drawstyle = Thin) override;
    void polygon(const vector<Vector2D>& points, const Color& c, int drawstyle = Filled) override;

//    void save(const char filename[50]);
};

#endif //SRC_VISUALIZATIONOPENCV_H
