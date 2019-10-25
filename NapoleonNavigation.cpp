#include <ros/ros.h>

#include <iostream>
#include <Visualization/Visualization.h>
#include <Definitions/Polygon.h>
#include <Model/HolonomicModel.h>
#include <Model/BicycleModel.h>
#include <Obstacles/Obstacles.h>
#include <Tube/Tube.h>
#include <Tube/Tubes.h>
#include <cmath>
#include <vector>
#include <Visualization/VisualizationOpenCV.h>
#include <Visualization/VisualizationRviz.h>
#include <Communication/Communication.h>

typedef Vector2D Vec;

#define wall(p1, p2) Obstacle(Polygon({p1, p2}, Open), Pose2D(p1, 0), Static)
#define staticobstacle(poly, middle, pose) Obstacle(Polygon(poly, middle, Closed), pose, Static)
#define dynamicobstacle(poly, pose) Obstacle(Polygon(poly), pose, Dynamic)

double F_loop = 30;
double F_prediction = 10;

int main(int argc, char** argv) {
    cout << "Main loop started" << endl;

    Polygon footprint({Vec(0,0), Vec(2,0), Vec(2, 0.2), Vec(2.3,0.2), Vec(2.3,0.8), Vec(2,0.8), Vec(2,1), Vec(0,1)}, Closed, true, Pose2D(1,0.5,0));
    //Polygon footprint({Vec(0,0), Vec(0.65,0), Vec(0.65,0.6), Vec(0,0.6)}, Closed, true, Pose2D(0.325,0.3,0));
    HolonomicModel hmodel(Pose2D(-2,-1,M_PI_2), footprint, 1, 1, 1);

    Tubes tubes;
//    Tubes tubes(Tube(Vec(60,32), 2, Vec(2,34), 2, 1));
//    tubes.addPoint(Vec(0,4), 2, 1);
//    tubes.addPoint(Vec(0,3), 2, 1);

    ros::init(argc, argv, "route_navigation");

    ros::NodeHandle nroshndl("~");
    ros::Rate rate(F_loop);

    VisualizationRviz canvas(nroshndl);
    Communication comm(nroshndl);

    bool startNavigation = false;
    while(!startNavigation){
        if(ros::ok()) {
            ros::spinOnce();
            if(comm.initialized) {
                hmodel.update(1/F_loop, comm);
                hmodel.show(canvas, Color(0, 0, 0), Thin);
                hmodel.showCommunicationInput(canvas, Color(0, 0, 0), Thin, comm);
                comm.obstacles.show(canvas, Color(255,0,0), Thick);

//                Vector2D p1 = hmodel.dilatedFootprint.boundingBoxRotated(hmodel.pose.a)[1];
//                Pose2D v1 = Pose2D(-0.5, 0.05, 0);
//                Pose2D i1 = hmodel.translateInput(p1, v1);
//                canvas.arrow(p1, p1+v1, Color(0,0,255),Thin);
//                hmodel.input(i1, Frame_World);

                tubes.showSides(canvas);
                canvas.resetId();
                if(comm.newPlan()) {
                    tubes.convertRoute(comm.route, hmodel, canvas);
                    startNavigation = true;
                }
            }
        }
        hmodel.update(1/F_loop, comm);
        rate.sleep();
    }

    FollowStatus realStatus = Status_Ok;

    while(nroshndl.ok() && ros::ok() && realStatus != Status_Done){

        canvas.checkId();
        canvas.resetId();

        canvas.arrow(Vec(0,0),Vec(1,0),Color(0,0,0),Thin);
        canvas.arrow(Vec(0,0),Vec(0,1),Color(0,0,0),Thin);

        HolonomicModel hmodelCopy = hmodel;
        FollowStatus predictionStatus = hmodelCopy.predict(10, 4, 0.3, 1/F_prediction, hmodel, tubes, canvas); //nScaling | predictionTime | minDistance
        hmodel.copySettings(hmodelCopy);

        //status = Status_Ok;

        if(predictionStatus == Status_Ok || predictionStatus == Status_Done) {
            realStatus = hmodel.follow(tubes, canvas, false);
            tubes.avoidObstacles(hmodel.currentTubeIndex, hmodel.currentTubeIndex, comm.obstacles, hmodel, DrivingSide_Right, canvas);
            switch (realStatus){
                case Status_ToClose: {cout << "Status To Close" << endl; break;}
                case Status_Stuck: {cout << "Status Stuck" << endl; break;}
                case Status_Error: {cout << "Status Error" << endl; break;}
                case Status_Collision: {cout << "Status Collision" << endl; break;}
                case Status_Done: {cout << "Status Done" << endl; break;}
            }
        }else{
            hmodel.input(Pose2D(0,0,0), Frame_Robot);
            switch (predictionStatus){
                case Status_ToClose: {cout << "Prediction status To Close" << endl; break;}
                case Status_Stuck: {cout << "Prediction status Stuck" << endl; break;}
                case Status_Error: {cout << "Prediction status Error" << endl; break;}
                case Status_Collision: {cout << "Prediction status Collision" << endl; break;}
            }
        }

        tubes.showSides(canvas);
        hmodel.show(canvas, Color(0,0,0), Thin);
        comm.obstacles.show(canvas, Color(255,0,0), Thick);

        hmodel.update(1/F_loop, comm);

        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown();
    return 0;
}

// Measure time >>>>>>>>>>>>>>>>>>
//long long m1 = cv::getTickCount();
//Code
//long long m2 = cv::getTickCount();
//cout << (1000*(m2-m1))/cv::getTickFrequency() << " ms" << endl;
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
