#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ropod_ros_msgs/RoutePlannerAction.h>
#include <ros/ros.h>

#include <iostream>
#include <Visualization/Visualization.h>
#include <Definitions/Polygon.h>
#include <Model/HolonomicModel.h>
#include <Model/BicycleModel.h>
#include <Obstacles/Obstacle.h>
#include <Tube/Tube.h>
#include <Tube/Tubes.h>
#include <cmath>
#include <vector>
#include <Visualization/VisualizationOpenCV.h>

VisualizationOpenCV canvas(1000,1000,50);

typedef Vector2D Vec;

#define wall(p1, p2) Obstacle(Polygon({p1, p2}, Open), Pose2D(p1, 0), Static)
#define staticobstacle(poly, middle, pose) Obstacle(Polygon(poly, middle, Closed), pose, Static)
#define dynamicobstacle(poly, pose) Obstacle(Polygon(poly), pose, Dynamic)

ropod_ros_msgs::RoutePlannerResult route;
bool newRoute = false;
int F = 30;

void getDebugRoutePlanCallback(const ropod_ros_msgs::RoutePlannerResultConstPtr& routeData){
    route = *routeData;
    ROS_INFO("new debug plan received");
    newRoute = true;
}

int main(int argc, char** argv) {
    cout << "Main loop started" << endl;

    //Polygon footprint({Vec(0,0), Vec(2,0), Vec(2, 0.2), Vec(2.3,0.2), Vec(2.3,0.8), Vec(2,0.8), Vec(2,1), Vec(0,1)}, Closed, true, Pose2D(1,0.5,0));
    Polygon footprint({Vec(0,0), Vec(0.65,0), Vec(0.65,0.6), Vec(0,0.6)}, Closed, true, Pose2D(0.325,0.3,0));
    HolonomicModel hmodel(Pose2D(-2,-1,M_PI_2), footprint, 0.5, 3, 0.25);

    vector<Obstacle> obstacles;
    //obstacles.emplace_back(dynamicobstacle((Circle(Vec(),0.5).toPoints(8)), Pose2D(1.5,1,0)));


    Tubes tubes(Tube(Vec(59.7,32.3), 2.5, Vec(59.7,60), 2.5, 1));
    tubes.addPoint(Vec(30,60), 2.5, 1);

    vector<Obstacle> walls;
    walls.emplace_back(wall(Vec(-1,-1), Vec(-1,9)));


    ros::init(argc, argv, "route_navigation");

    ros::NodeHandle nroshndl("~");
    ros::Rate rate(F);

    hmodel.subscribe(nroshndl);

    //ros::Subscriber goal_cmd_sub = nroshndl.subscribe<geometry_msgs::PoseStamped>("/route_navigation/simple_goal", 10, simpleGoalCallback);
    ros::Subscriber ropod_debug_plan_sub = nroshndl.subscribe< ropod_ros_msgs::RoutePlannerResult >("/ropod/debug_route_plan", 1, getDebugRoutePlanCallback);

    while(!newRoute){
        if(ros::ok()) {
            ros::spinOnce();
        }
        rate.sleep();
    }

    while(nroshndl.ok()){
        canvas.setorigin(Pose2D(hmodel.pose.x, hmodel.pose.y, 0)-canvas.getWindowMidOffset());
        canvas.emptycanvas();
        canvas.arrow(Vec(0,0),Vec(1,0),Color(0,0,0),Thin);
        canvas.arrow(Vec(0,0),Vec(0,1),Color(0,0,0),Thin);

        HolonomicModel hmodelCopy = hmodel;
        FollowStatus status = hmodelCopy.predict(20, 2, 1, 1.0/F, hmodel, tubes, canvas); //nScaling | predictionTime | minDistance
        hmodel.copySettings(hmodelCopy);

        //status = Status_Ok;
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

        canvas.visualize();

        hmodel.update(1.0/F);
        //hmodel.setSpeed(Pose2D(0,0.1,0));

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

// Measure time >>>>>>>>>>>>>>>>>>
//long long m1 = cv::getTickCount();
//Code
//long long m2 = cv::getTickCount();
//cout << (1000*(m2-m1))/cv::getTickFrequency() << " ms" << endl;
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
