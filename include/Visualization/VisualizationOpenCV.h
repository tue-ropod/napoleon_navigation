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

enum ElementType {Element_Point, Element_Line, Element_Circle, Element_Arrow, Element_Rectangle, Element_Polygon};

class Element{

public:
    int id;
    ElementType elementType;
    DrawStyle drawStyle;
    Color color;
    vector<Vector2D> points;
    double radius;

    Element(int id_, ElementType elementType_, DrawStyle drawStyle_, Color color_, vector<Vector2D> points_, double radius_){
        id = id_;
        elementType = elementType_;
        drawStyle = drawStyle_;
        color = color_;
        points = std::move(points_);
        radius = radius_;
    }
};

class VisualizationOpenCV : public Visualization{
protected:
    vector<Element> elements;
    double pixelsPerMeter;
    cv::Mat canvas;
    //cv::Mat staticCanvas;
    int width;
    int height;
    Pose2D origin = Pose2D(0,0,0);

public:
    VisualizationOpenCV(int width_, int height_, int pixelsPerMeter_);

    int isElement(Element element);
    void addElement(Element element);
    void removeElement(int index);
    void removeDynamicElements();

    void setorigin(const Pose2D& origin_);
    Pose2D getWindowMidOffset();
    void emptycanvas();
    void visualize();
    cv::Point worldToCanvas(const Vector2D& p);
    void point(const Vector2D& p, const Color& c, unsigned int thickness) override;
    void line(const Vector2D& p1, const Vector2D& p2, const Color& c, unsigned int thickness) override;
    void arrow(const Vector2D& p1, const Vector2D& p2, const Color& c, unsigned int thickness) override;
    void circle(const Vector2D& p, double radius, const Color& c, int drawstyle) override;
    void rectangle(const Vector2D &p1, const Vector2D &p2, const Color &c, int drawstyle) override;
    void polygon(const vector<Vector2D>& points, const Color& c, int drawstyle) override;
};

#endif //SRC_VISUALIZATIONOPENCV_H
