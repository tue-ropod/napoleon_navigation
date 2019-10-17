//
// Created by bob on 17-10-19.
//

#ifndef SRC_VISUALIZATIONRVIZ_H
#define SRC_VISUALIZATIONRVIZ_H

#include <Definitions/Pose2D.h>
#include <Visualization/Visualization.h>


using namespace std;

class VisualizationRviz : public Visualization{
private:


public:
    VisualizationRviz();
    void point(const Vector2D& p, const Color& c, unsigned int thickness = Thin) override;
    void line(const Vector2D& p1, const Vector2D& p2, const Color& c, unsigned int thickness = Thin) override;
    void arrow(const Vector2D& p1, const Vector2D& p2, const Color& c, unsigned int thickness = Thin) override;
    void circle(const Vector2D& p, double radius, const Color& c, int drawstyle = Thin) override;
    void rectangle(const Vector2D &p1, const Vector2D &p2, const Color &c, int drawstyle = Thin) override;
    void polygon(const vector<Vector2D>& points, const Color& c, int drawstyle = Filled) override;
};


#endif //SRC_VISUALIZATIONRVIZ_H
