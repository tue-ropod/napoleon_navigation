//
// Created by bob on 17-10-19.
//

#ifndef SRC_VISUALIZATIONRVIZ_H
#define SRC_VISUALIZATIONRVIZ_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Definitions/Pose2D.h>
#include <Visualization/Visualization.h>

using namespace std;

class VisualizationRviz : public Visualization{
private:
    ros::Publisher visualization_pub;
    visualization_msgs::Marker baseMarker;

public:
    VisualizationRviz(ros::NodeHandle nroshndl);
    void removeAll();
    void checkId();
    void point(const Vector2D& p, const Color& c, unsigned int thickness) override;
    void line(const Vector2D& p1, const Vector2D& p2, const Color& c, unsigned int thickness) override;
    void arrow(const Vector2D& p1, const Vector2D& p2, const Color& c, unsigned int thickness) override;
    void circle(const Vector2D& p, double radius, const Color& c, int drawstyle) override;
    void rectangle(const Vector2D &p1, const Vector2D &p2, const Color &c, int drawstyle) override;
    void polygon(const vector<Vector2D>& points, const Color& c, int drawstyle) override;
};


#endif //SRC_VISUALIZATIONRVIZ_H
