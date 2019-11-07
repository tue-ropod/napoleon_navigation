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

    //Polygon footprint({Vec(0,0), Vec(2,0), Vec(2, 0.2), Vec(2.3,0.2), Vec(2.3,0.8), Vec(2,0.8), Vec(2,1), Vec(0,1)}, Closed, true, Pose2D(1,0.5,0));
    Polygon footprint({Vec(0,0), Vec(0.65,0), Vec(0.65,0.65), Vec(0,0.65)}, Closed, true, Pose2D(0.325,0.325,0));
    HolonomicModel hmodel(Pose2D(0,0,M_PI_2), footprint, 1.5, 0.7, 0.25);

    Tubes tubes;
//    Tubes tubes(Tube(Vec(0,0), 1, Vec(-3,5), 1, 1));
//    tubes.addPoint(Vec(3,5), 1, 1);
//    tubes.addPoint(Vec(3,0.5), 1, 1);
//    tubes.addPoint(Vec(3,0), 1, 1);
    bool testRoute = false;

    ros::init(argc, argv, "route_navigation");

    ros::NodeHandle nroshndl("~");
    ros::Rate rate(F_loop);

    VisualizationRviz canvas(nroshndl);
    Communication comm(nroshndl);

    bool initialized = false;
    while(!initialized){
        if(ros::ok()) {
            ros::spinOnce();
            if(comm.initialized) {
                initialized = true;
            }
        }
        rate.sleep();
    }

    FollowStatus realStatus = Status_Ok;
    FollowStatus predictionStatus = Status_Ok;
    bool startNavigation = false;
    HolonomicModel hmodelCopy = hmodel;

    while(nroshndl.ok() && ros::ok() && comm.initialized){

        canvas.checkId();
        canvas.resetId();

        //tubes.visualizePlan(comm.route, canvas);
        tubes.visualizeRightWall(comm.route, canvas);

        canvas.arrow(Vec(0,0),Vec(1,0),Color(0,0,0),Thin);
        canvas.arrow(Vec(0,0),Vec(0,1),Color(0,0,0),Thin);

        if(startNavigation){
            if(realStatus == Status_Recovering){
                tubes.recover(hmodel);
            }

            predictionStatus = hmodelCopy.predict(10, 4, 1, 1/F_prediction, hmodel, tubes, comm.obstacles, canvas); //nScaling | predictionTime | minDistance

            if(predictionStatus == Status_Ok || predictionStatus == Status_Done || predictionStatus == Status_ShortPredictionDistance || predictionStatus == Status_TubeCollision || predictionStatus == Status_Recovering) {
                hmodel.copySettings(hmodelCopy);
                realStatus = hmodel.follow(tubes, canvas, true);
                //tubes.avoidObstacles(hmodel.currentTubeIndex, hmodel.currentTubeIndex, obstacles, hmodel, DrivingSide_Right, canvas);
                if(realStatus != Status_Ok && realStatus != Status_TubeCollision) {hmodel.brake();}
            }
            else if(predictionStatus == Status_ObstacleCollision){
                hmodel.input(Pose2D(0,0,0),Frame_World);
            }
            else{
                hmodel.brake();
            }

            if(realStatus == Status_Done) {
                hmodel.brake();
                startNavigation = false;
            }
        }
        if(!startNavigation && comm.newPlan()) {
            if(tubes.convertRoute(comm.route, hmodel, canvas)){
                startNavigation = true;
            }else{
                cout << "Tube is larger than the defined area! [check if margins can be decreased or if the object fits through at all.]" << endl;
            }
        }

        hmodelCopy.showStatus("Prediction model");
        hmodel.showStatus("Model");
        tubes.showSides(canvas);
        hmodel.show(canvas, Color(0,0,0), Thin);
        hmodel.showCommunicationInput(canvas, Color(0,255,50), Thin, comm);
        comm.obstacles.show(canvas, Color(255,0,0), Thick);

        double ct = rate.cycleTime().toSec();
        double ect = rate.expectedCycleTime().toSec();
        double cycleTime = ct > ect ? ct : ect;

        hmodel.update(cycleTime, comm);

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
