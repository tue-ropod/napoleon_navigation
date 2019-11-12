//
// Created by bob on 25-09-19.
//

#ifndef NAVIGATION_MODEL_H
#define NAVIGATION_MODEL_H

#include <utility>
#include <vector>
#include <Definitions/Polygon.h>
#include <Definitions/Pose2D.h>
#include <Visualization/Visualization.h>
#include <Obstacles/Obstacle.h>
#include <Tube/Tubes.h>
#include <Communication/Communication.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>


class Tubes;

enum Movement {Movement_Forward, Movement_Backward, Movement_Left, Movement_Right, Movement_RotateLeft, Movement_RotateRight};
enum Frame {Frame_Robot, Frame_World};
enum FollowStatus {Status_Ok, Status_ObstacleCollision, Status_TubeCollision, Status_Stuck, Status_Error, Status_Done, Status_ShortPredictionDistance, Status_OutsideTube, Status_Recovering, Status_WrongWay};

class Model {
protected:

public:
    double maxSpeed;
    double maxAcceleration;
    double maxRotationalSpeed;
    double maxRotationalAcceleration;
    double footprintClearance = 0.05;
    int currentTubeIndex = 0;
    vector<Movement> limitedMovements;

    Polygon footprint;
    Polygon dilatedFootprint;
    Pose2D pose = Pose2D(0,0,0), velocity = Pose2D(0,0,0);
    Pose2D inputVelocity = Pose2D(0,0,0), desiredVelocity = Pose2D(0,0,0), predictionBiasVelocity = Pose2D(0,0,0);
    bool applyBrake = false;
    bool poseInitialized = false;
    FollowStatus status = Status_Ok, prevStatus = Status_Error;
    double speedScale = 1;

    Model(Pose2D pose_, Polygon footprint_, double maxSpeed_, double maxAcceleration_, double wheelDistanceToMiddle_);
    bool checkCollision(Obstacles& o);
    void dilateFootprint(double offset);
    void update(double dt, Communication &comm);
    void brake();
    void changeSpeedScale(double x);
    void copySettings(Model &modelCopy);
    void copyState(Model &modelCopy);
    double width();
    double length();
    double turnWidth();
    double brakeDistance();
    FollowStatus predict(double dt, Model &origionalModel, Tubes &tubes, Communication &comm, Visualization &canvas);
    void showCommunicationInput(Visualization& canvas, Color c, int drawstyle, Communication &comm);
    void showStatus(const string& modelName);

    virtual void show(Visualization& canvas, Color c, int drawstyle);
    virtual FollowStatus follow(Tubes& tubes, Communication& comm, Visualization& canvas, bool debug);
    virtual void updateModel(double dt);
    virtual void updatePrediction(double dt);
    virtual void input(Pose2D velocity_, Frame frame);
    virtual Pose2D translateInput(Vector2D position, Pose2D velocity_);

};

#endif //NAVIGATION_MODEL_H
