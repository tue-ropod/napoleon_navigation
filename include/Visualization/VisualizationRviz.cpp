//
// Created by bob on 17-10-19.
//

#include "VisualizationRviz.h"

VisualizationRviz::VisualizationRviz(ros::NodeHandle nroshndl){
    visualization_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_navigation", 100, false);

    baseMarker.lifetime = ros::Duration(1);
    baseMarker.header.frame_id = "map";
    baseMarker.ns = "napoleon";
    baseMarker.action = visualization_msgs::Marker::ADD;
    baseMarker.pose.orientation.w = 1.0;
    baseMarker.scale.z = 0.01;

}

void VisualizationRviz::removeAll(){
    visualization_msgs::Marker marker = baseMarker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    visualization_pub.publish(marker);
}

void VisualizationRviz::point(const Vector2D& p, const Color& c, unsigned int thickness){
    if(p.valid()) {
        visualization_msgs::Marker marker = baseMarker;
        marker.header.stamp = ros::Time::now();

        geometry_msgs::Point point;
        point.x = p.x;
        point.y = p.y;
        marker.points.push_back(point);

        marker.id = getId(idName);
        marker.ns += ("/"+idName);
        marker.type = visualization_msgs::Marker::POINTS;

        marker.scale.x = 0.01 * thickness;
        marker.scale.y = 0.01 * thickness;

        marker.color.a = c.alpha / 255.0;
        marker.color.r = c.red / 255.0;
        marker.color.g = c.green / 255.0;
        marker.color.b = c.blue / 255.0;

        visualization_pub.publish(marker);
    }
}

void VisualizationRviz::line(const Vector2D& p1, const Vector2D& p2, const Color& c, unsigned int thickness){
    if(p1.valid() && p2.valid()) {
        visualization_msgs::Marker marker = baseMarker;
        marker.header.stamp = ros::Time::now();

        geometry_msgs::Point point1, point2;
        point1.x = p1.x;
        point1.y = p1.y;
        point2.x = p2.x;
        point2.y = p2.y;
        marker.points.push_back(point1);
        marker.points.push_back(point2);

        marker.id = getId(idName);
        marker.ns += ("/"+idName);
        marker.type = visualization_msgs::Marker::LINE_LIST;

        marker.scale.x = 0.01 * thickness;
        marker.scale.y = 0.01 * thickness;

        marker.color.a = c.alpha / 255.0;
        marker.color.r = c.red / 255.0;
        marker.color.g = c.green / 255.0;
        marker.color.b = c.blue / 255.0;

        visualization_pub.publish(marker);
    }
}

void VisualizationRviz::arrow(const Vector2D& p1, const Vector2D& p2, const Color& c, unsigned int thickness){
    if(p1.valid() && p2.valid()) {
        visualization_msgs::Marker marker = baseMarker;
        marker.header.stamp = ros::Time::now();

        geometry_msgs::Point point1, point2;
        point1.x = p1.x;
        point1.y = p1.y;
        point2.x = p2.x;
        point2.y = p2.y;
        marker.points.push_back(point1);
        marker.points.push_back(point2);

        marker.id = getId(idName);
        marker.ns += ("/"+idName);
        marker.type = visualization_msgs::Marker::ARROW;

        marker.scale.x = 0.01 * thickness;
        marker.scale.y = 0.03 * thickness;
        marker.scale.z = 0;

        marker.color.a = c.alpha / 255.0;
        marker.color.r = c.red / 255.0;
        marker.color.g = c.green / 255.0;
        marker.color.b = c.blue / 255.0;

        visualization_pub.publish(marker);
    }
}

void VisualizationRviz::circle(const Vector2D& p, double radius, const Color& c, int drawstyle){
    if(p.valid()) {
        visualization_msgs::Marker marker = baseMarker;
        marker.header.stamp = ros::Time::now();

        geometry_msgs::Point point;
        point.x = p.x;
        point.y = p.y;
        marker.points.push_back(point);

        marker.id = getId(idName);
        marker.ns += ("/"+idName);
        marker.type = visualization_msgs::Marker::POINTS;

        marker.scale.x = radius / 2;
        marker.scale.y = radius / 2;

        marker.color.a = c.alpha / 255.0;
        marker.color.r = c.red / 255.0;
        marker.color.g = c.green / 255.0;
        marker.color.b = c.blue / 255.0;

        visualization_pub.publish(marker);
    }
}

void VisualizationRviz::rectangle(const Vector2D &p1, const Vector2D &p2, const Color &c, int drawstyle){
    if(p1.valid() && p2.valid()) {
        visualization_msgs::Marker marker = baseMarker;
        marker.header.stamp = ros::Time::now();

        for (int r = 0; r < 5; r++) {
            geometry_msgs::Point point;
            switch (r) {
                case 0:
                case 5:
                    point.x = p1.x;
                    point.y = p1.y;
                    break;
                case 1:
                    point.x = p2.x;
                    point.y = p1.y;
                    break;
                case 2:
                    point.x = p2.x;
                    point.y = p2.y;
                    break;
                case 3:
                    point.x = p1.x;
                    point.y = p2.y;
                    break;
            }
            marker.points.push_back(point);
        }

        marker.id = getId(idName);
        marker.ns += ("/"+idName);
        marker.type = visualization_msgs::Marker::LINE_STRIP;

        marker.scale.x = 0.01 * abs(drawstyle);

        marker.color.a = c.alpha / 255.0;
        marker.color.r = c.red / 255.0;
        marker.color.g = c.green / 255.0;
        marker.color.b = c.blue / 255.0;

        visualization_pub.publish(marker);
    }
}

void VisualizationRviz::polygon(const vector<Vector2D>& points, const Color& c, int drawstyle){
    bool valid = true;
    for(auto & p : points){if( !p.valid() ){valid = false;}}
    if(valid && !points.empty()) {
        visualization_msgs::Marker marker = baseMarker;
        marker.header.stamp = ros::Time::now();

        geometry_msgs::Point point;
        for (int i = 0; i < points.size() + 1; i++) {
            Vector2D p = points[i % points.size()];
            point.x = p.x;
            point.y = p.y;
            marker.points.push_back(point);
        }

        marker.id = getId(idName);
        marker.ns += ("/"+idName);
        marker.type = visualization_msgs::Marker::LINE_STRIP;

        marker.scale.x = 0.01 * abs(drawstyle);

        marker.color.a = c.alpha / 255.0;
        marker.color.r = c.red / 255.0;
        marker.color.g = c.green / 255.0;
        marker.color.b = c.blue / 255.0;

        visualization_pub.publish(marker);
    }
}

void VisualizationRviz::lines(const vector<Vector2D>& points, const Color& c, int drawstyle){
    bool valid = true;
    for(auto & p : points){if( !p.valid() ){valid = false;}}
    if(valid && !points.empty()) {
        visualization_msgs::Marker marker = baseMarker;
        marker.header.stamp = ros::Time::now();

        geometry_msgs::Point point;
        for (int i = 0; i < points.size(); i++) {
            Vector2D p = points[i % points.size()];
            point.x = p.x;
            point.y = p.y;
            marker.points.push_back(point);
        }

        marker.id = getId(idName);
        marker.ns += ("/"+idName);
        marker.type = visualization_msgs::Marker::LINE_LIST;

        marker.scale.x = 0.01 * abs(drawstyle);

        marker.color.a = c.alpha / 255.0;
        marker.color.r = c.red / 255.0;
        marker.color.g = c.green / 255.0;
        marker.color.b = c.blue / 255.0;

        visualization_pub.publish(marker);
    }
}