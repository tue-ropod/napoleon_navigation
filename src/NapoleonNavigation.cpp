#include <ros/ros.h>

#include <iostream>
#include <Visualization/Visualization.h>
#include <Definitions/Polygon.h>
#include <Model/HolonomicModel.h>
#include <Tube/Tubes.h>
#include <Visualization/VisualizationRviz.h>
#include <Communication/Communication.h>

double F_loop = 30;
double F_prediction = 10;

int main(int argc, char** argv) {
    cout << "Main loop started" << endl;

    ros::init(argc, argv, "route_navigation");

    ros::NodeHandle nroshndl("~");
    ros::Rate rate(F_loop);

    VisualizationRviz canvas(nroshndl);
    Communication comm(nroshndl);

    Obstacles obstacles;
    Tubes tubes;
    Polygon footprint(comm.footprint_param, Closed, true, comm.footprintMiddlePose_param);
    HolonomicModel hmodel(Pose2D(0,0,0), footprint, comm.maxSpeed_param, comm.maxAcceleration_param, comm.wheelDistanceMiddle_param);

    comm.obstacles = &obstacles;

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
    FollowStatus predictionStatus;
    bool startNavigation = false;
    bool update = false;
    HolonomicModel hmodelCopy = hmodel;

    while(nroshndl.ok() && ros::ok() && comm.initialized){

        canvas.resetId();

        tubes.visualizePlan(comm.route, canvas);
        tubes.visualizeRightWall(comm.route, canvas);

        canvas.idName = "Frame";
        canvas.arrow(Vector2D(0,0),Vector2D(1,0),Color(0,0,0),Thin);
        canvas.arrow(Vector2D(0,0),Vector2D(0,1),Color(0,0,0),Thin);

        canvas.idName = "AMCL_uncertainty";
        canvas.polygon(comm.poseUncertainty.toPoints(10), Color(100,100,100), Thin);

        obstacles.removeOldVisibleObstacles(hmodel.scanArea);
        obstacles.removeOldSelectiveObstacles();

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
                //if(realStatus != Status_Ok && realStatus != Status_TubeCollision) {hmodel.brake();}
            }
            else{
                hmodel.brake();
            }

            if(realStatus == Status_Done) {
                hmodel.brake();
                startNavigation = false;
            }
            update = true;
        }
        if(!startNavigation) {
            if(comm.newPlan()) {
                if (tubes.convertRoute(comm, hmodel, canvas)) {
                    startNavigation = true;
                    cout << "Tube ready." << endl;
                } else {
                    cout << "Tube is larger than the defined area! [check if margins can be decreased or if the object fits through at all.]" << endl;
                }
            }
        }
        hmodel.checkCollision(obstacles, canvas);

        hmodelCopy.showStatus("Prediction model");
        hmodel.showStatus("Model");
        tubes.showSides(canvas);
        hmodel.show(canvas, Color(0,0,0), Thin);
        hmodel.showCommunicationInput(canvas, Color(0,255,50), Thin, comm);
        obstacles.show(canvas, Color(255,0,0), Thick);

        double ct = rate.cycleTime().toSec();
        double ect = rate.expectedCycleTime().toSec();
        double cycleTime = ct > ect ? ct : ect;
        hmodel.update(cycleTime, comm, update);
        obstacles.update(cycleTime);
        update = false;

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
