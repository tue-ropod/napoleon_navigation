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

    ros::init(argc, argv, "route_navigation");

    ros::NodeHandle nroshndl("~");
    ros::Rate rate(F_loop);

    VisualizationRviz canvas(nroshndl);
    Communication comm(nroshndl);

    Tubes tubes;
    Polygon footprint(comm.footprint_param, Closed, true, comm.footprintMiddlePose_param);
    HolonomicModel hmodel(Pose2D(0,0,0), footprint, comm.maxSpeed_param, comm.maxAcceleration_param, comm.wheelDistanceMiddle_param);

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

    FollowStatus realStatus = Status_Error;
    FollowStatus predictionStatus;
    bool startNavigation = false;
    bool stopped = false;
    HolonomicModel hmodelCopy = hmodel;

    while(nroshndl.ok() && ros::ok() && comm.initialized){

        canvas.resetId();

        tubes.visualizePlan(comm.route, canvas);
        tubes.visualizeRightWall(comm.route, canvas);

        canvas.idName = "Frame";
        canvas.arrow(Vec(0,0),Vec(1,0),Color(0,0,0),Thin);
        canvas.arrow(Vec(0,0),Vec(0,1),Color(0,0,0),Thin);

        canvas.idName = "AMCL_uncertainty";
        canvas.polygon(comm.poseUncertainty.toPoints(10), Color(100,100,100), Thin);

        if(startNavigation){
            if(realStatus == Status_Recovering){
                //tubes.recover(hmodel);
            }
            if(realStatus == Status_WrongWay){
                //rotate model
            }

            predictionStatus = hmodelCopy.predict(1/F_prediction, hmodel, tubes, comm, canvas); //nScaling | predictionTime | minDistance

            if( predictionStatus == Status_Ok ||
                predictionStatus == Status_Done ||
                predictionStatus == Status_ShortPredictionDistance ||
                predictionStatus == Status_TubeCollision ||
                predictionStatus == Status_Recovering ||
                predictionStatus == Status_WrongWay ||
                predictionStatus == Status_ObstacleCollision            ){

                hmodel.copySettings(hmodelCopy);
                int temp = hmodel.currentTubeIndex;
                realStatus = hmodel.follow(tubes, comm, canvas, true);
                if(hmodel.currentTubeIndex != temp){cout << "Pose in tube "<< hmodel.currentTubeIndex << endl;}

                //tubes.avoidObstacles(hmodel.currentTubeIndex, hmodel.currentTubeIndex, obstacles, hmodel, DrivingSide_Right, canvas);
//                if(realStatus != Status_Ok && realStatus != Status_TubeCollision) {hmodel.brake();}
            }
            else{
                hmodel.brake();
            }

            if(realStatus == Status_Done) {
                hmodel.brake();
                startNavigation = false;
            }
            stopped = false;
        }
        if(!startNavigation) {
            hmodel.brake();
            if(comm.newPlan()) {
                if (tubes.convertRoute(comm, hmodel, canvas)) {
                    startNavigation = true;
                    cout << "Tube ready." << endl;
                } else {
                    cout << "Tube is larger than the defined area! [check if margins can be decreased or if the object fits through at all.]" << endl;
                }
            }
        }
        hmodel.checkCollision(comm.obstacles, canvas);

        hmodelCopy.showStatus("Prediction model");
        hmodel.showStatus("Model");
        tubes.showSides(canvas);
        hmodel.show(canvas, Color(0,0,0), Thin);
        hmodel.showCommunicationInput(canvas, Color(0,255,50), Thin, comm);
        comm.obstacles.show(canvas, Color(255,0,0), Thick);

		if(startNavigation || !stopped){
			double ct = rate.cycleTime().toSec();
			double ect = rate.expectedCycleTime().toSec();
			double cycleTime = ct > ect ? ct : ect;
			hmodel.update(cycleTime, comm);
			stopped = true;
		}
		
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
