#include <iostream>
#include <Visualization/Visualization.h>
#include <Definitions/Polygon.h>
#include <Model/HolonomicModel.h>
#include <Model/BicycleModel.h>
#include <Obstacles/Obstacle.h>
#include <LoopRate/LoopRate.h>
#include <Tube/Tube.h>
#include <Tube/Tubes.h>
#include <Visualization/VisualizationOpenCV.h>

VisualizationOpenCV canvas(1000,1000,60);

typedef Vector2D Vec;

#define wall(p1, p2) Obstacle(Polygon({p1, p2}, Open), Pose2D(p1, 0), Static)
#define staticobstacle(poly, middle, pose) Obstacle(Polygon(poly, middle, Closed), pose, Static)
#define dynamicobstacle(poly, pose) Obstacle(Polygon(poly), pose, Dynamic)

int main() {
    //canvas.setorigin(Pose2D(-6,-6,0));

    Polygon footprint({Vec(0,0), Vec(2,0), Vec(2, 0.2), Vec(2.3,0.2), Vec(2.3,0.8), Vec(2,0.8), Vec(2,1), Vec(0,1)}, Closed, true, Pose2D(1,0.5,0));
    //Polygon footprint({Vec(0,0), Vec(0.65,0), Vec(0.65,0.6), Vec(0,0.6)}, Closed, true, Pose2D(0.325,0.3,0));
    HolonomicModel hmodel(Pose2D(-2,-1,M_PI_2), footprint, 6, 8, 1);

    vector<Obstacle> obstacles;
    obstacles.emplace_back(dynamicobstacle((Circle(Vec(),0.5).toPoints(8)), Pose2D(1.5,1,0)));
    //obstacles.emplace_back(dynamicobstacle((Circle(Vec(),0.5).toPoints(8)), Pose2D(1,10,0)));
    //obstacles.emplace_back(dynamicobstacle((Circle(Vec(),0.5).toPoints(8)), Pose2D(9,9,0)));
    //obstacles.emplace_back(dynamicobstacle((Circle(Vec(),0.5).toPoints(8)), Pose2D(9,4,0)));

    Tubes tubes(Tube(Vec(-3,-4), 2.5, Vec(-2,0), 2.5, 1));
    tubes.addPoint(Vec(-3,8), 2, 1);
    tubes.addPoint(Vec(8,9), 2, 1);
    tubes.addPoint(Vec(8,3), 1.7, 0.6);
    tubes.addPoint(Vec(2,5), 1.7, 0.4);
    tubes.addPoint(Vec(2,-4), 1.7, 0.4);
    tubes.addPoint(Vec(11,-4), 2, 0.6);
    tubes.addPoint(Vec(11,12), 1.6, 1);
    tubes.addPoint(Vec(-2,12), 1.3, 0.5);
    tubes.addPoint(Vec(-3,12), 1.3, 0);

    vector<Obstacle> walls;
    walls.emplace_back(wall(Vec(-1,-1), Vec(-1,9)));

    LoopRate r(50);

    while(true){
        canvas.setorigin(Pose2D(hmodel.pose.x, hmodel.pose.y, 0)-canvas.getWindowMidOffset());

        canvas.emptycanvas();

        canvas.arrow(Vec(0,0),Vec(1,0),Color(0,0,0),Thin);
        canvas.arrow(Vec(0,0),Vec(0,1),Color(0,0,0),Thin);

        //obstacles[0].movement = Pose2D(0.2*cos((M_PI*2.0*r.elapsedTime)/(1000*5)), 0.3*cos((M_PI*2.0*r.elapsedTime)/(1000*7)), 0.01);

        HolonomicModel hmodelCopy = hmodel;
        FollowStatus status = hmodelCopy.predict(20, 2, 1, r.periodSeconds, hmodel, tubes, canvas); //nScaling | predictionTime | minDistance
        hmodel.copySettings(hmodelCopy);

        if(status == Status_Ok || status == Status_Done) {
            FollowStatus realStatus = hmodel.follow(tubes, canvas, true);
            tubes.avoidObstacles(hmodel.currentTubeIndex, hmodel.currentTubeIndex, obstacles, hmodel, DrivingSide_Right, canvas);
            if(realStatus == Status_Done){
                cout << "Status Done" << endl;
                cout << "Exit Simulation" << endl;
                break;
            }
        }else{
            hmodel.input(Pose2D(0,0,0), Frame_Robot);
            switch (status){
                case Status_ToClose: {cout << "Status To Close" << endl; break;}
                case Status_Stuck: {cout << "Status Stuck" << endl; break;}
                case Status_Error: {cout << "Status Error" << endl; break;}
                case Status_Collision: {cout << "Status Collision" << endl; break;}
            }
            cout << "Exit Simulation" << endl;
            break;
        }

        tubes.showSides(canvas);
        hmodel.show(canvas, Color(0,0,0), Thin);

        for(auto & obstacle : obstacles){
            obstacle.show(canvas, Color(200,200,200), Filled);
        }

        canvas.visualize();

        //obstacles[0].update();
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
