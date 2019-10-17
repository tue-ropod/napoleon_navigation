//
// Created by bob on 17-10-19.
//

#include "VisualizationOpenCV.h"

Visualization::Visualization(int width_, int height_, int pixelsPerMeter_){
    pixelsPerMeter = pixelsPerMeter_;
    width = width_;
    height = height_;
    emptycanvas();
}

void Visualization::setorigin(const Pose2D& origin_){
    origin = origin_;
}

Pose2D Visualization::getWindowMidOffset() {
    return {(width/(2.0*pixelsPerMeter)), (height/(2.0*pixelsPerMeter)), 0};
}

void Visualization::visualize(){
    cv::imshow("Visualization", canvas);
    cv::waitKey(3);
}

void Visualization::emptycanvas(){
    cv::Mat newcanvas(height, width, CV_8UC4, cv::Scalar(80,80,80));
    canvas = newcanvas;
}

cv::Point Visualization::worldToCanvas(const Vector2D& p){
    Vector2D pt = p;
    pt.transformThis(0, 0, -origin.a);
    pt.transformThis(-origin.x, -origin.y, 0);
    return cv::Point(pt.x * pixelsPerMeter, -pt.y * pixelsPerMeter) + cv::Point(0,canvas.rows);
}

void Visualization::point(const Vector2D &p, const Color &c, unsigned int thickness){
    if(c.alpha != 255){
        cv::Mat overlay;
        canvas.copyTo(overlay);
        cv::circle(overlay, worldToCanvas(p), (int)thickness, cv::Scalar(c.blue, c.green, c.red), -1);
        cv::addWeighted(overlay, (double)c.alpha/255, canvas, 1 - ((double)c.alpha/255), 0, canvas);
    }else {
        cv::circle(canvas, worldToCanvas(p), (int)thickness, cv::Scalar(c.blue, c.green, c.red), -1);
    }
}

void Visualization::line(const Vector2D &p1, const Vector2D &p2, const Color &c, unsigned int thickness) {
    if(c.alpha != 255){
        cv::Mat overlay;
        canvas.copyTo(overlay);
        cv::line(overlay, worldToCanvas(p1), worldToCanvas(p2), cv::Scalar(c.blue, c.green, c.red), (int)thickness);
        cv::addWeighted(overlay, (double)c.alpha/255, canvas, 1 - ((double)c.alpha/255), 0, canvas);
    }else {
        cv::line(canvas, worldToCanvas(p1), worldToCanvas(p2), cv::Scalar(c.blue, c.green, c.red), (int)thickness);
    }
}

void Visualization::arrow(const Vector2D &p1, const Vector2D &p2, const Color &c, unsigned int thickness) {
    double length = p1.distance(p2);
    Vector2D tip = p1;
    tip = tip - p2;
    tip.unitThis();
    tip.scaleThis(length / 4, length / 4);
    Vector2D A1 = tip;
    Vector2D A2 = tip;
    A1.transformThis(p2.x, p2.y, M_PI / 8);
    A2.transformThis(p2.x, p2.y, -M_PI / 8);
    if(c.alpha != 255){
        cv::Mat overlay;
        canvas.copyTo(overlay);
        cv::line(overlay, worldToCanvas(p1), worldToCanvas(p2), cv::Scalar(c.blue, c.green, c.red), (int)thickness);
        cv::line(overlay, worldToCanvas(A1), worldToCanvas(p2), cv::Scalar(c.blue, c.green, c.red), (int)thickness);
        cv::line(overlay, worldToCanvas(A2), worldToCanvas(p2), cv::Scalar(c.blue, c.green, c.red), (int)thickness);
        cv::addWeighted(overlay, (double)c.alpha/255, canvas, 1 - ((double)c.alpha/255), 0, canvas);
    }else {
        cv::line(canvas, worldToCanvas(p1), worldToCanvas(p2), cv::Scalar(c.blue, c.green, c.red), (int)thickness);
        cv::line(canvas, worldToCanvas(A1), worldToCanvas(p2), cv::Scalar(c.blue, c.green, c.red), (int)thickness);
        cv::line(canvas, worldToCanvas(A2), worldToCanvas(p2), cv::Scalar(c.blue, c.green, c.red), (int)thickness);
    }
}

void Visualization::circle(const Vector2D &p, const double radius, const Color &c, int drawstyle) {
    if(c.alpha != 255){
        cv::Mat overlay;
        canvas.copyTo(overlay);
        cv::circle(overlay, worldToCanvas(p), (int)(radius * pixelsPerMeter), cv::Scalar(c.blue, c.green, c.red), drawstyle);
        cv::addWeighted(overlay, (double)c.alpha/255, canvas, 1 - ((double)c.alpha/255), 0, canvas);
    }else {
        cv::circle(canvas, worldToCanvas(p), (int)(radius * pixelsPerMeter), cv::Scalar(c.blue, c.green, c.red), drawstyle);
    }
}

void Visualization::rectangle(const Vector2D &p1, const Vector2D &p2, const Color &c, int drawstyle) {
    if(c.alpha != 255){
        cv::Mat overlay;
        canvas.copyTo(overlay);
        cv::rectangle(overlay, worldToCanvas(p1), worldToCanvas(p2), cv::Scalar(c.blue, c.green, c.red), drawstyle);
        cv::addWeighted(overlay, (double)c.alpha/255, canvas, 1 - ((double)c.alpha/255), 0, canvas);
    }else {
        cv::rectangle(canvas, worldToCanvas(p1), worldToCanvas(p2), cv::Scalar(c.blue, c.green, c.red), drawstyle);
    }
}

void Visualization::polygon(const vector<Vector2D> &points, const Color &c, int drawstyle) {
    const int npts = points.size();
    if(drawstyle == Filled){
        cv::Point pts [npts];
        for(int i = 0; i < npts; i++){
            pts[i] = worldToCanvas(points[i]);
        }
        if(c.alpha != 255){
            cv::Mat overlay;
            canvas.copyTo(overlay);
            cv::fillConvexPoly(overlay, pts, npts, cv::Scalar(c.blue, c.green, c.red));
            cv::addWeighted(overlay, (double)c.alpha/255, canvas, 1 - ((double)c.alpha/255), 0, canvas);
        }else {
            cv::fillConvexPoly(canvas, pts, npts, cv::Scalar(c.blue, c.green, c.red));
        }
    } else{
        for(int i = 0; i < npts; i++){
            Vector2D p1 = points[i];
            Vector2D p2 = points[(i+1)%npts];
            line(p1, p2, c, drawstyle);
        }
    }
}