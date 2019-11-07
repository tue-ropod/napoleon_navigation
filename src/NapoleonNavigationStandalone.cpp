#include <iostream>
#include <Visualization/Visualization.h>
#include <Definitions/Polygon.h>
#include <Model/HolonomicModel.h>
#include <Model/BicycleModel.h>
#include <Obstacles/Obstacles.h>
#include <LoopRate/LoopRate.h>
#include <Tube/Tube.h>
#include <Tube/Tubes.h>
#include <Visualization/VisualizationOpenCV.h>

VisualizationOpenCV canvas(1000,1000,40);

typedef Vector2D Vec;

#define wall(p1, p2) Obstacle(Polygon({p1, p2}, Open), Pose2D(p1, 0), Static)
#define staticobstacle(poly, middle, pose) Obstacle(Polygon(poly, middle, Closed), pose, Static)
#define dynamicobstacle(poly, pose) Obstacle(Polygon(poly), pose, Dynamic)

int main() {
    double F_loop = 30;
    double F_planner = 10;

    //canvas.setorigin(Pose2D(-10,-8,0));

    Polygon footprint({Vec(0,0), Vec(2,0), Vec(2, 0.2), Vec(2.3,0.2), Vec(2.3,0.8), Vec(2,0.8), Vec(2,1), Vec(0,1)}, Closed, true, Pose2D(1,0.5,0));
    //Polygon footprint({Vec(0,0), Vec(0.65,0), Vec(0.65,0.6), Vec(0,0.6)}, Closed, true, Pose2D(0.325,0.3,0));
    HolonomicModel hmodel(Pose2D(-5,-4,M_PI_2), footprint, 3, 0.8, 1);

    Obstacles obstacles;
    //obstacles.obstacles.emplace_back(dynamicobstacle((Circle(Vec(),0.5).toPoints(8)), Pose2D(1.5,1,0)));
    //obstacles.obstacles.emplace_back(dynamicobstacle((Circle(Vec(),0.5).toPoints(8)), Pose2D(1,10,0)));
    //obstacles.obstacles.emplace_back(dynamicobstacle((Circle(Vec(),0.5).toPoints(8)), Pose2D(9,9,0)));
    //obstacles.obstacles.emplace_back(dynamicobstacle((Circle(Vec(),0.5).toPoints(8)), Pose2D(9,4,0)));

    Tubes tubes(Tube(Vec(-3,-6), 2, Vec(-3,-2), 2, 1));
    tubes.addPoint(Vec(-3,-1), 1.5, 1);
    tubes.addPoint(Vec(-3,8), 2, 1);
    tubes.addPoint(Vec(8,9), 2, 1);
    tubes.addPoint(Vec(8,5), 1.7, 0.6);
    tubes.addPoint(Vec(2,5), 1.7, 0.7);
    tubes.addPoint(Vec(2,-4), 1.7, 0.7);
    tubes.addPoint(Vec(11,-4), 2, 0.6);
    tubes.addPoint(Vec(11,12), 1.6, 1);
    tubes.addPoint(Vec(0,12), 1.6, 0.5);
    tubes.addPoint(Vec(-3,12), 1.6, 0.5);
    tubes.addPoint(Vec(-6,12), 4, 0.3);

    LoopRate r(F_loop);

    FollowStatus realStatus = Status_Ok;
    FollowStatus predictionStatus = Status_Ok;
    HolonomicModel hmodelCopy = hmodel;


    while(realStatus != Status_Done){
        canvas.setorigin(Pose2D(hmodel.pose.x, hmodel.pose.y, 0)-canvas.getWindowMidOffset());

        canvas.emptycanvas();

        canvas.arrow(Vec(0,0),Vec(1,0),Color(0,0,0),Thin);
        canvas.arrow(Vec(0,0),Vec(0,1),Color(0,0,0),Thin);

        if(realStatus == Status_Recovering){
            tubes.recover(hmodel);
        }

        predictionStatus = hmodelCopy.predict(10, 4, 1, 1/F_planner, hmodel, tubes, obstacles, canvas); //nScaling | predictionTime | minDistance

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

        hmodelCopy.showStatus("Prediction model");
        hmodel.showStatus("Model");
        tubes.showSides(canvas);
        hmodel.show(canvas, Color(0,0,0), Thin);
        obstacles.show(canvas, Color(100,100,100), Filled);

        canvas.visualize();

        hmodel.updatePrediction(r.periodSeconds);

        r.update();
    }
    return 0;
}

// Measure time >>>>>>>>>>>>>>>>>>
//long long m1 = cv::getTickCount();
//Code
//long long m2 = cv::getTickCount();
//cout << (1000*(m2-m1))/cv::getTickFrequency() << " ms" << endl;
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
