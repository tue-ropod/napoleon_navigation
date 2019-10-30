#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <Visualization/Visualization.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <vector>
#include <Visualization/VisualizationOpenCV.h>
#include <Visualization/VisualizationRviz.h>
#include <Communication/Communication.h>

geometry_msgs::PointStamped clickedPoint;
bool clicked = false;

void pointCallback(const geometry_msgs::PointStampedConstPtr& point){
    clickedPoint = *point;
    clicked = true;
    cout << "New Point" << endl;
}

int main(int argc, char** argv) {
    cout << "Main loop started" << endl;

    ros::init(argc, argv, "point_builder");
    ros::NodeHandle nroshndl("~");
    ros::Rate rate(30);

    ros::Subscriber clickedPoint_sub = nroshndl.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, pointCallback);

    int id = 0;
    std::ofstream myfile;
    bool first = true;
    while(nroshndl.ok() && ros::ok()){

        if(clicked){
            clicked = false;
            if(first) {
                myfile.open("/home/bob/Documents/mapPoints/map_points.txt");
                first = false;
            }
            myfile << "-" <<"\n";
            myfile << "\t x: " << clickedPoint.point.x << "\n";
            myfile << "\t y: " << clickedPoint.point.y << "\n";
            myfile << "\t id: " << id << "\n";
            id++;
        }

        ros::spinOnce();
        rate.sleep();
    }
    myfile.close();
    ros::shutdown();
    return 0;
}

// Measure time >>>>>>>>>>>>>>>>>>
//long long m1 = cv::getTickCount();
//Code
//long long m2 = cv::getTickCount();
//cout << (1000*(m2-m1))/cv::getTickFrequency() << " ms" << endl;
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
