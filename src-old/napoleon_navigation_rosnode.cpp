#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <ed_gui_server/objPosVel.h>
#include <ed_gui_server/objsPosVel.h>
#include <std_msgs/Bool.h>
#include <ropod_ros_msgs/GoToAction.h>
#include <ropod_ros_msgs/RoutePlannerAction.h>

#include <math.h>
#include <stdlib.h>
#include <fstream>
#include <ctime>
#include <string>
#include <vector>
#include <algorithm>    // std::rotate

#include "napoleon_config.h"
#include "napoleon_geometry.h"
#include "napoleon_functions.h"
#include "napoleon_action.h"
#include "napoleon_assignment.h"
#include "napoleon_model.h"
#include "napoleon_obstacle.h"
#include "napoleon_prediction.h"
#include "napoleon_tube.h"
#include "napoleon_visualization.h"

double program_duration = 0, real_time_est = 0;
bool start_navigation = false;
bool ropod_reached_target = false;
bool goal_received = false;

//std::vector<geometry_msgs::PoseStamped> global_path;
geometry_msgs::PoseStamped simple_goal;

ros::Publisher mapmarker_pub;
ros::Publisher wallmarker_pub;
ros::Publisher ropodmarker_pub;

ropod_ros_msgs::RoutePlannerResult debug_route_planner_result_;

void simpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
    ROS_INFO("new simple goal received");
    start_navigation = true;
}

void getDebugRoutePlanCallback(const ropod_ros_msgs::RoutePlannerResultConstPtr& result)
{
    debug_route_planner_result_ = *result;
    ROS_INFO("new debug plan received");
    start_navigation = true;
}

int main(int argc, char** argv)
{
    cout << "Main loop started" << endl;
    ros::init(argc, argv, "route_navigation");

    NapoleonModel Model;
    NapoleonPrediction Prediction;
    NapoleonAssignment Assignment;
    NapoleonObstacle Obstacle;
    NapoleonVisualization Visualization;
    NapoleonAction Action;

    ros::NodeHandle nroshndl("~");
    ros::Rate rate(F_PLANNER);

    ros::Subscriber goal_cmd_sub = nroshndl.subscribe<geometry_msgs::PoseStamped>("/route_navigation/simple_goal", 10, simpleGoalCallback);
    ros::Subscriber amcl_pose_sub = nroshndl.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &NapoleonModel::getAmclPoseCallback, &Model);
    ros::Subscriber ropod_odom_sub = nroshndl.subscribe<nav_msgs::Odometry>("/ropod/odom", 100, &NapoleonModel::getOdomVelCallback, &Model);
    ros::Subscriber ropod_debug_plan_sub = nroshndl.subscribe< ropod_ros_msgs::RoutePlannerResult >("/ropod/debug_route_plan", 1, getDebugRoutePlanCallback);
    ros::Subscriber scan_sub = nroshndl.subscribe<sensor_msgs::LaserScan>("/ropod/laser/scan", 1, &NapoleonModel::scanCallback, &Model);

//    ros::Subscriber obstacle_sub = nroshndl.subscribe<ed_gui_server::objsPosVel>("/ed/gui/objectPosVel", 10, getObstaclesCallback);
    ros::Publisher vel_pub = nroshndl.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Visualize map nodes and robot
    ropodmarker_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_driving/ropodpoints", 1);
    mapmarker_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_driving/vmnodes", 100, true);
    wallmarker_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_driving/right_side_wall", 10, true);

    ROS_INFO("Wait for debug plan on topic");

    while(ros::ok())
    {
        if(start_navigation)
        {
            break;
        }
        ros::spinOnce();
    }

    ROS_INFO("Initialize Assignment");
    Assignment.initializeAssignment(debug_route_planner_result_.areas);

    ROS_INFO("Initialize Visualization");
    Visualization.initializeVisualizationMarkers(Assignment);

    ROS_INFO("Open log file");
    std::ofstream myfile;
    //myfile.open ("/simdata/ropod_" + get_date() +".txt");
    myfile.open ("/home/bob/Documents/simdata/ropod_" + get_date() +".txt");
    myfile << "time" << "\t" << "tictoc" << "\t" << "state" << "\t" << "task counter" << "\t" << "tube width" << "\t" <<
            "phi des" << "\t" << "v ropod odom" << "\t"<< "v ropod cmd" << "\t" << "x ropod" << "\t" << "y ropod" << "\t" <<
            "theta ropod" << "\t" "x obs" << "\t" << "y obs" << "\t" << "theta obs" << "\t" << "obs width" << "\t" <<
            "obs depth" << "\t" << "obs vx" << "\t" << "obs vy" << "\t" << "des accel" <<"\n";

    std::clock_t start_loop;

    ROS_INFO("Now starting navigation");

    while(nroshndl.ok() && !ropod_reached_target)
    {
        ROS_INFO("Process scan data");
        Model.processLatestScanData(); //Process scan data

        if (start_navigation){

            start_loop = std::clock();

            ROS_INFO("Start prediction");
            Prediction.start(Model);

            ROS_INFO("Update position");
            Model.updatePosition(Prediction); //model update position
    
            //ROS_INFO("Ropod x: %f / Ropod y: %f / Theta: %f", ropod_x, ropod_y, ropod_theta);
            //ROS_INFO("xdot: %f / ydot: %f / thetadot %f", odom_xdot_ropod_global, odom_ydot_ropod_global, odom_thetadot_global);
            //ROS_INFO("ropodx: %f / ropody: %f / ropodtheta %f", ropod_x, ropod_y, ropod_theta)
    
            while ((Model.ropod_colliding_obs || Model.ropod_colliding_wall) && Prediction.k < V_SCALE_OPTIONS.size())
            {
                Prediction.initialize(Model, Obstacle); //initialize prediction
                Assignment.initializeAreas(Prediction); //initialize areas
                Prediction.update(ropodmarker_pub, wallmarker_pub, Assignment, Model, Obstacle, Visualization, Prediction); //predict
            }// end while finding v_scale
    
            // Update after a prediction is made where no collision is caused
            // The prediction is ran until t_min_pred, however, the ropod will run a
            // new prediction the next step, so only the first part of the
            // prediction is used.

            Prediction.step(); //prediction step

            program_duration = ( std::clock() - start_loop ) / (double) CLOCKS_PER_SEC;
            real_time_est = real_time_est+1/F_PLANNER;
    
            Model.updateControlVelocity(Prediction); //update control velocity
    
//            myfile << real_time_est << "\t" << program_duration << "\t" << pred_state[0] << "\t" << pred_task_counter[0] << "\t" << pred_tube_width[0] << "\t" <<
//                pred_phi_des[0] << "\t" << pred_v_ropod[0] << "\t" << control_v << "\t"  <<  pred_xy_ropod[0].x << "\t" <<  pred_xy_ropod[0].y << "\t" <<
//                pred_plan_theta[0] << "\t" << pred_x_obs[0] << "\t" << pred_y_obs[0] << "\t" << obs_theta << "\t" << current_obstacle.width << "\t" <<
//                current_obstacle.depth << "\t" << current_obstacle.vel.x << "\t" << current_obstacle.vel.y << "\t" << pred_accel[1] << "\t" <<"\n";
    
            ropod_reached_target = Prediction.checkIfFinished(Assignment);//prediction check finished

            Visualization.publish(ropodmarker_pub, Model, Prediction, Assignment);//visualization publish
            
        }   // end if received goal

        ros::spinOnce();
        rate.sleep();
        if(rate.cycleTime() > ros::Duration(1/F_PLANNER) ){
            ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", F_PLANNER, rate.cycleTime().toSec());
        }
    }
    // Will only perform this when ropod has reached target

    Action.stop(vel_pub);//action stop

    myfile.close();

    return 0;
}
