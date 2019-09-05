#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <queue>
#include <string>
#include <vector>
#include "napoleon_config.h"
#include "napoleon_geometry.h"
#include "napoleon_functions.h"
#include <algorithm>    // std::rotate
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <ctime>

#include <ed_gui_server/objPosVel.h>
#include <ed_gui_server/objsPosVel.h>

#include <actionlib/server/simple_action_server.h>
#include <ropod_ros_msgs/GoToAction.h>
#include <ropod_ros_msgs/RoutePlannerAction.h>

#include <math.h>

#include <nav_msgs/Path.h>

#include <std_msgs/Bool.h>
#include <stdlib.h>

std::vector<PointID> pointlist;
std::vector<AreaQuadID> arealist;
std::vector<int> assignment;

double ropod_x = 0, ropod_y = 0, ropod_theta = 0;
double this_amcl_x = 0, this_amcl_y = 0, quaternion_x = 0, quaternion_y = 0, quaternion_z = 0, quaternion_w = 0, this_amcl_theta = 0, siny_cosp = 0, cosy_cosp = 0;
double odom_xdot_ropod_global = 0, odom_ydot_ropod_global = 0, odom_thetadot_global = 0, odom_phi_local = 0, odom_phi_global = 0, odom_vropod_global = 0;
int no_obs = 0;
ed_gui_server::objPosVel current_obstacle;
double obs_theta;
Point obs_center_global;
double program_duration = 0, real_time_est = 0;

//std::vector<geometry_msgs::PoseStamped> global_path;
bool start_navigation = false;
geometry_msgs::PoseStamped simple_goal;

bool goal_received = false;



//void getObstaclesCallback(const ed_gui_server::objsPosVel::ConstPtr& obsarray) {
//    no_obs = obsarray->objects.size();
//    //ROS_INFO("%d obstacles detected", no_obs);
//    //string s;
//    // For the sake of proof of concept, the assumption is made that there will
//    // only be one obstacle to avoid or overtake.
//    // This scenario is not realistic and only serves as showcase.
//    // A counter will decide which obstacle to choose
//    // for (int q = 0; q < no_obs; ++q) {
//    //     s = obsarray->objects[q].id;
//    //     std::cout << s << std::endl;
//    //     current_obstacle = obsarray->objects[q];
//    // }
//    // For now just pick first obs if there is obs
//    // so we assume we only see the obstacle we want to see
//    if (no_obs > 0) {
//        // If no other obstacle is seen, this obstacle is kept
//        current_obstacle = obsarray->objects[0];
//        // ROS_INFO("Obs is %f wide and %f deep", current_obstacle.width, current_obstacle.depth);
//        // ROS_INFO("Obs x: %f, obs y: %f", current_obstacle.pose.position.x, current_obstacle.pose.position.y);
//        // ROS_INFO("Vx: %f, Vy %f", current_obstacle.vel.x, current_obstacle.vel.y);
//        quaternion_x = obsarray->objects[0].pose.orientation.x;
//        quaternion_y = obsarray->objects[0].pose.orientation.y;
//        quaternion_z = obsarray->objects[0].pose.orientation.z;
//        quaternion_w = obsarray->objects[0].pose.orientation.w;
//
//        // yaw (z-axis rotation)
//        siny_cosp = +2.0 * (quaternion_w * quaternion_z + quaternion_x * quaternion_y);
//        cosy_cosp = +1.0 - 2.0 * (quaternion_y * quaternion_y + quaternion_z * quaternion_z);
//        obs_theta = atan2(siny_cosp, cosy_cosp);
//        obs_center_global.x = current_obstacle.pose.position.x;
//        obs_center_global.y = current_obstacle.pose.position.y;
//    }
//}

void getOdomVelCallback(const nav_msgs::Odometry::ConstPtr& odom_vel)
{
    odom_xdot_ropod_global = odom_vel->twist.twist.linear.x;
    odom_ydot_ropod_global = odom_vel->twist.twist.linear.y;
    odom_thetadot_global = odom_vel->twist.twist.angular.z;
    odom_phi_local = atan2(odom_ydot_ropod_global, odom_xdot_ropod_global);
    odom_vropod_global = sqrt(odom_xdot_ropod_global*odom_xdot_ropod_global+odom_ydot_ropod_global*odom_ydot_ropod_global);
    //ROS_INFO("xdot: %f, ydot: %f, vabs: %f", odom_xdot_ropod_global, odom_ydot_ropod_global, odom_vropod_global);
}

void getAmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
    //ROS_INFO("Amcl pose received");
    //ROS_INFO("X: %f, Y: %f", pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
    this_amcl_x = pose_msg->pose.pose.position.x;
    this_amcl_y = pose_msg->pose.pose.position.y;
    quaternion_x = pose_msg->pose.pose.orientation.x;
    quaternion_y = pose_msg->pose.pose.orientation.y;
    quaternion_z = pose_msg->pose.pose.orientation.z;
    quaternion_w = pose_msg->pose.pose.orientation.w;

    // yaw (z-axis rotation)
    siny_cosp = +2.0 * (quaternion_w * quaternion_z + quaternion_x * quaternion_y);
    cosy_cosp = +1.0 - 2.0 * (quaternion_y * quaternion_y + quaternion_z * quaternion_z);
    this_amcl_theta = atan2(siny_cosp, cosy_cosp);
}

void simpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
    ROS_INFO("new simple goal received");
    start_navigation = true;
}

std::queue<sensor_msgs::LaserScan::ConstPtr> scan_buffer_;
bool scan_available = false;
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_available = true;
    scan_buffer_.push(msg);
}

sensor_msgs::LaserScan::ConstPtr scan;
tf::TransformListener* tf_listener_;
std::vector<geometry_msgs::Point> laser_meas_points;

void getLatestScanData()
{

    while(!scan_buffer_.empty())
    {
        scan = scan_buffer_.front();

        // - - - - - - - - - - - - - - - - - -
        // Determine absolute laser pose based on TF

        try
        {
            tf::StampedTransform t_sensor_pose;
            tf_listener_->lookupTransform("map", scan->header.frame_id, scan->header.stamp, t_sensor_pose);
            scan_buffer_.pop();

            tf::Quaternion q = t_sensor_pose.getRotation(); //( t_sensor_pose.getRotation().x, t_sensor_pose.getRotation().y, t_sensor_pose.getRotation().z, t_sensor_pose.getRotation().w );
            tf::Matrix3x3 matrix ( q );
            double rollSensor, pitchSensor, yawSensor;
            matrix.getRPY ( rollSensor, pitchSensor, yawSensor );

            double scan_size =  scan->ranges.size();
            laser_meas_points.resize(scan_size);

            for(unsigned int iScan = 0; iScan < scan_size; iScan ++)
            {
                double angle = yawSensor + scan->angle_min + scan->angle_increment*iScan;
                laser_meas_points[iScan].x = t_sensor_pose.getOrigin().getX() + scan->ranges[iScan]*cos( angle );
                laser_meas_points[iScan].y = t_sensor_pose.getOrigin().getY() + scan->ranges[iScan]*sin( angle );
            }
        }
        catch(tf::ExtrapolationException& ex)
        {
            ROS_WARN_STREAM_DELAYED_THROTTLE(10, "ED Laserplugin tracking: " << ex.what());
            try
            {
                // Now we have to check if the error was an interpolation or extrapolation error
                // (i.e., the scan is too old or too new, respectively)
                tf::StampedTransform latest_transform;
                tf_listener_->lookupTransform("map", scan->header.frame_id, ros::Time(0), latest_transform);

                if (scan_buffer_.front()->header.stamp > latest_transform.stamp_)
                {
                    // Scan is too new
                    break;
                }
                else
                {
                    // Otherwise it has to be too old (pop it because we cannot use it anymore)
                    scan_buffer_.pop();
                }
            }
            catch(tf::TransformException& exc)
            {
                scan_buffer_.pop();
            }
        }
        catch(tf::TransformException& exc)
        {
            ROS_ERROR_STREAM_DELAYED_THROTTLE(10, "ED Laserplugin tracking: " << exc.what());
            scan_buffer_.pop();
        }
    }
}

void showWallPoints(Point local_wallpoint_front, Point local_wallpoint_rear,  ros::Publisher &pub) {
    //ROS_INFO_STREAM("showWallPoints (" << local_wallpoint_front.x  << ", " << local_wallpoint_front.y << "), ("
    //        << local_wallpoint_rear.x << ", " << local_wallpoint_rear.y << ")");
    visualization_msgs::Marker vis_wall;
    vis_wall.header.frame_id = "ropod/base_link";
    vis_wall.header.stamp = ros::Time::now();
    // vis_points.ns = line_strip.ns = line_list.ns = "points_in_map";
    vis_wall.action = visualization_msgs::Marker::ADD;
    vis_wall.pose.orientation.w = 1.0;
    vis_wall.id = 90;
    vis_wall.color.a = 0.7;
    vis_wall.color.g = 0.3;
    vis_wall.color.b = 0.5;
    vis_wall.type = visualization_msgs::Marker::POINTS;
    vis_wall.scale.x = 0.3;
    vis_wall.scale.y = 0.3;
    geometry_msgs::Point wall_p;

    vis_wall.points.clear();
    wall_p.x =  local_wallpoint_front.x;
    wall_p.y =  local_wallpoint_front.y;
    vis_wall.points.push_back(wall_p);
    wall_p.x =  local_wallpoint_rear.x;
    wall_p.y =  local_wallpoint_rear.y;
    vis_wall.points.push_back(wall_p);
    pub.publish(vis_wall);
}

/******************************************
 * Visualization markers
 ******************************************/
visualization_msgs::Marker vis_points;
visualization_msgs::Marker vis_wall;
visualization_msgs::Marker vis_plan;

void initializeVisualizationMarkers()
{
    // Visualize map nodes
    vis_points.header.frame_id = "/map";
    vis_points.header.stamp = ros::Time::now();
    // vis_points.ns = line_strip.ns = line_list.ns = "points_in_map";
    vis_points.action = visualization_msgs::Marker::ADD;
    vis_points.pose.orientation.w = 1.0;
    vis_points.id = 0;
    vis_points.color.a = 1.0;
    vis_points.color.g = 1.0;
    vis_points.type = visualization_msgs::Marker::POINTS;
    vis_points.scale.x = 0.2;
    vis_points.scale.y = 0.2;
    geometry_msgs::Point vis_p;

    for (int imap = 0; imap < pointlist.size(); ++imap)
    {
        vis_p.x = pointlist[imap].x;
        vis_p.y = pointlist[imap].y;
        vis_p.z = 0;
        vis_points.points.push_back(vis_p);
    }

    // End visualize map nodes

    // Visualize wall the ropod is following
    vis_wall.header.frame_id = "/map";
    vis_wall.header.stamp = ros::Time::now();
    // vis_points.ns = line_strip.ns = line_list.ns = "points_in_map";
    vis_wall.action = visualization_msgs::Marker::ADD;
    vis_wall.pose.orientation.w = 1.0;
    vis_wall.id = 100;
    vis_wall.color.a = 0.7;
    vis_wall.color.g = 0.3;
    vis_wall.color.b = 0.5;
    vis_wall.type = visualization_msgs::Marker::POINTS;
    vis_wall.scale.x = 0.3;
    vis_wall.scale.y = 0.3;

        // Visualize wall the ropod is following
    vis_plan.header.frame_id = "/map";
    vis_plan.header.stamp = ros::Time::now();
    // vis_points.ns = line_strip.ns = line_list.ns = "points_in_map";
    vis_plan.action = visualization_msgs::Marker::ADD;
    vis_plan.pose.orientation.w = 1.0;
    vis_plan.id = 1;
    vis_plan.color.a = 0.7;
    vis_plan.color.g = 0.3;
    vis_plan.color.b = 0.5;
    vis_plan.type = visualization_msgs::Marker::LINE_STRIP;
    vis_plan.scale.x = 0.3;
    vis_plan.scale.y = 0.3;

}

/*********************************************************/


/******************************************
 * Global scope variables
 ******************************************/

// Initial values, not very important as long as ropod is in the right area
// First action will be determined by the initial values (or I can just wait?)
double control_v = 0;
double theta_0 = 0.0;       // Initial orientation of ropod [rad]
double x_ropod_0 = 0.0;     // X position of center of ropod [m]
double y_ropod_0 = 0.0;     // Y position of center of ropod [m]
double v_ropod_0 = 0.0;     // Velocity of ropod [m/s]
double phi_0 = 0.00;        // Steering angle - CCW positive - 0 when steering straight [rad]
double phi_dot_0 = 0.0;     // Steering velocity [rad/s]
double x_rearax_0 = 0.0;    // X position of center of rear axle [m]
double y_rearax_0 = 0.0;    // Y position of center of rear axle [m]
double x_rearax;            // X position of center of rear axle [m]
double y_rearax;            // Y position of center of rear axle [m]
double prev_amcl_x = 0;     // Used to update position with amcl or 'predict' new pose
double prev_amcl_y = 0;
double prev_amcl_theta = 0;

static constexpr int size_m = ((int)F_MODEL)*(T_MAX_PRED+1)+1;  // Array size for predictions that run at F_MODEL
static constexpr int size_p = ((int)F_PLANNER)*(T_MAX_PRED+1)+1;   // Array size for predictions that run at F_PLANNER
double t_pred[size_m] {0};              // Prediction time [s]
double t_pred_j[size_p] {0};              // Prediction time planning [s]
double pred_v_ropod[size_m] {v_ropod_0};
double pred_v_ropod_plan[size_p] {pred_v_ropod[0]};
bool pred_ropod_on_entry_inter[size_p] {false};
bool pred_ropod_on_entry_hall[size_p] {false};
double pred_x_ropod[size_p] {x_ropod_0};
double pred_y_ropod[size_p] {y_ropod_0};
double pred_x_obs[size_p] {0};
double pred_y_obs[size_p] {0};
double v_obs_sq = 0;
double prev_sim_phi_des;
double pred_plan_theta[size_p] {theta_0};
double pred_accel[size_p] {0};
std::vector<double> sim_theta {pred_plan_theta[0]};
std::vector<double> sim_phi {phi_0};
std::vector<double> sim_v_scale {1};
double prev_sim_tube_width {TUBE_WIDTH_C};
std::vector<std::string> walls;
double pred_phi[size_m];
double pred_theta[size_m];
double pred_xdot[size_m];
double pred_ydot[size_m];
double pred_thetadot[size_m];
double pred_x_rearax[size_m];
double pred_y_rearax[size_m];
double prev_pred_phi_des;
double pred_phi_des[size_p] {0};
double pred_tube_width[size_p] {TUBE_WIDTH_C};
Point pred_ropod_rb, pred_ropod_lb, pred_ropod_rt, pred_ropod_lt,
pred_ropod_dil_rb, pred_ropod_dil_lb, pred_ropod_dil_lt, pred_ropod_dil_rt,
obslt, obsrt, obsrb, obslb;
Point vis_rt, vis_lt, vis_fr, vis_fl;
bool update_state_points;
bool dir_cw;
double v_des_scaled[size_p] {0};
double vel_dif, v_new;
double t_pred_prev;
double dist_to_middle_final;
bool pred_ropod_colliding_obs[size_p] {false};
Point current_obs_in_ropod_frame_pos, obs_in_ropod_frame_pos;

PointID rw_p_rear, rw_p_front, lw_p_rear, lw_p_front;

AreaQuadID cur_obj;

vector<string> areaIDs;
int ind, qmax;
double space_left, space_right, shift_wall, wallang;;

int prev_sim_state = 1;
int pred_state[size_p] {1};
int prev_sim_task_counter = 0;
int pred_task_counter[size_p] {0};
int danger_count = 0; // Counter that ensures program stops if collision with wall predicted for sometime

// Constants during simulation
double delta_t = 1/(double)F_PLANNER;                      // Time ropod will execute this plan
double max_delta_v = A_MAX*delta_t;             // Maximum change in v in delta_t
double lpf = fmax(1.0,2*M_PI*TS*CUTOFF_FREQ);             // Low pass filter [-]
bool ropod_reached_target = false;

// Counters
int i = 0; // - simulation/experiment plan
int n = 1; // - simulation/experiment movement
int k = -1; //- velocity scaling
int q = 1; // - for loop for movement simulation (in prediction)
int j = 0; // - prediction plan
int m = 0; // - prediction movement
int u = 0; // - Pred task counter
int prevstate; // Actually just j-1
int m_prev;
int ka_max;  // Assignment length
double v_ax = 0, theta_dot = 0, v_des, phi, v_scale;

int delta_assignment_on_overtake;
bool consider_overtaking_current_hallway, consider_overtaking_next_hallway;
bool update_assignment;
int uprev;
bool ropod_colliding_obs = true;
bool ropod_colliding_wall = true;

AreaQuadID current_hallway;
AreaQuadID next_hallway;
AreaQuadID curr_area;
AreaQuadID next_area;
AreaQuadID next_second_area;

PointID current_pivot, cur_next_hallway_rear, cur_next_hallway_front;
PointID current_inter_rear_wall, current_inter_front_wall;
Point pred_xy_ropod_0(pred_x_ropod[0], pred_y_ropod[0]);
std::array<Point, size_p> pred_xy_ropod {{pred_xy_ropod_0}};
Point cur_pivot_local, cur_next_hallway_rear_local, cur_next_hallway_front_local;
Point current_inter_rear_wall_local, current_inter_front_wall_local;
double cur_next_hallway_angle;
PointID wall_front_p0, wall_front_p1;
Point local_wall_front_p0, local_wall_front_p1;
Point local_front_ropod_dilated_p0(DILATE_ROPOD_ALIGNING, -DILATE_ROPOD_ALIGNING);
Point local_front_ropod_dilated_p1(DILATE_ROPOD_ALIGNING, DILATE_ROPOD_ALIGNING);
PointID point_rear, point_front;
Point glob_wallpoint_front, glob_wallpoint_rear;
Point global_wall_front_p0, global_wall_front_p1;
PointID point_pivot;
Point local_wallpoint_front, local_wallpoint_rear;
Point local_pivot;

bool sharp_corner[100] = {false}; // quick but dirty fix! - now size is fixed to 100 since we don't
                                  // have number of areas value during compilation time
                                  // This will fail if number of areas go above 100


std::vector<std::vector<string>> OBJ_X_TASK;
std::vector<std::string> task1, task2, task3;
std::vector<std::string> current_hallway_task, next_hallway_task;

int area1ID, area2ID, area3ID;

PointID obj2wall_p0, obj2wall_p1, obj3wall_p0, obj3wall_p1;
double obj2frontwall_angle, obj3wall_angle, relative_angle;


/******************************************
 * Initialization fo assigment
 ******************************************/

void initializeAssignment()
{
    current_hallway = getAreaByID(assignment[0],arealist); // TODO: This assumes we start in a hallway, which could be not the case when stuck in an intersection
    for (int arID = 1; arID < ka_max; arID++)
    {
        if (arealist[arID].type == "hallway")
        {
            printf("AreaNumber initial next hallway: %d\n",arID);
            next_hallway = arealist[arID];
            break;
        }
    }

    ROS_INFO("Compute assigment nodes from assigment area vector:");

    std::vector<std::string> OBJ1TASK, OBJ2TASK;
    AreaQuadID OBJ1 = getAreaByID(assignment[0],arealist);
    AreaQuadID OBJ2 = getAreaByID(assignment[1],arealist);
    AreaQuadID OBJ3 = getAreaByID(assignment[2],arealist);
    int obj2tasklen;

    for (int ka = 0; ka < ka_max; ka = ka+1) {
        AreaQuadID curr_OBJ = getAreaByID(assignment[ka],arealist);
        std::vector<string> area_names;

        if(curr_OBJ.type=="hallway")
        {
            printf("Hallway assignment: ");
            OBJ1 = getAreaByID(assignment[ka],arealist);
            OBJ2 = getAreaByID(assignment[ka+1],arealist);
            OBJ3 = getAreaByID(assignment[ka+2],arealist);
            if (ka < (ka_max-1) )
            {
                OBJ1TASK = getWallPointsTowardsB(OBJ1,OBJ2);
            }
            else
            {
                OBJ1TASK = getWallPointsAwayFromB(OBJ1,getAreaByID(assignment[ka-1],arealist));
            }
            printstringvec(OBJ1TASK);
            area_names.push_back(OBJ1TASK[0]);
            area_names.push_back(OBJ1TASK[1]);
            area_names.push_back("");
            area_names.push_back("");
            area_names.push_back("");
            area_names.push_back("");
            OBJ_X_TASK.push_back(area_names);
        }
        else if(curr_OBJ.type=="inter")
        {
            printf("Intersection assignment: ");
            OBJ1 = getAreaByID(assignment[ka-1],arealist);
            OBJ2 = getAreaByID(assignment[ka],arealist);
            OBJ3 = getAreaByID(assignment[ka+1],arealist);
            OBJ2TASK = getPointsForTurning(OBJ1,OBJ2,OBJ3,OBJ1TASK);
            printstringvec(OBJ2TASK);
            obj2tasklen = OBJ2TASK.size();
            area_names.push_back(OBJ2TASK[0]);
            area_names.push_back(OBJ2TASK[1]);
            if (obj2tasklen == 6) {
                area_names.push_back(OBJ2TASK[2]);
                area_names.push_back(OBJ2TASK[3]);
                area_names.push_back(OBJ2TASK[4]);
                area_names.push_back(OBJ2TASK[5]);

                obj2wall_p0 = getPointByID(OBJ2TASK[0],pointlist);
                obj2wall_p1 = getPointByID(OBJ2TASK[1],pointlist);
                obj3wall_p0 = getPointByID(OBJ2TASK[2],pointlist);
                obj3wall_p1 = getPointByID(OBJ2TASK[3],pointlist);

                obj2frontwall_angle = atan2(obj2wall_p1.y-obj2wall_p0.y, obj2wall_p1.x-obj2wall_p0.x);
                obj3wall_angle = atan2(obj3wall_p1.y-obj3wall_p0.y, obj3wall_p1.x-obj3wall_p0.x);
                relative_angle = wrapToPi(obj3wall_angle-obj2frontwall_angle);

                if (OBJ2TASK[5].compare("right") == 0 && relative_angle < -SHARP_ANGLE_TRESHOLD) {
                    // Sharp angle to the right, we need to take the next wall into account as well
                    sharp_corner[ka+1] = true;
                } else if (OBJ2TASK[5].compare("left") == 0 && relative_angle > SHARP_ANGLE_TRESHOLD) {
                    // Sharp angle to the left, we need to take the next wall into account as well
                    sharp_corner[ka+1] = true;
                }
            }
            else
            {
                area_names.push_back("");
                area_names.push_back("");
                area_names.push_back("");
                area_names.push_back("");

            }
            OBJ_X_TASK.push_back(area_names);
        }

    }
}


void considerOvertaking()
{
    // Consider overtaking.
    // Cesar -> TODO: Because of the changes to support consecutive hallway areas, this part still needs to be adapted
    // Cesar -> TODO: The contains function should consider not only the center but all corners. Also how to deal with multiple obstacles?
    if (u < ka_max-1 && no_obs > 0) {
        if (update_assignment) {
            if(curr_area.type == "hallway"){
                current_hallway = curr_area;
                current_hallway_task = task1;
                if (current_hallway.contains(obs_center_global)) {
                    consider_overtaking_current_hallway = true;
                    // This strategy only allows to overtake 1 obstacle
                }
            }else if(next_area.type == "hallway")
            {
                next_hallway = next_area;
                next_hallway_task = task2;
                delta_assignment_on_overtake = 1;
                if (next_hallway.contains(obs_center_global)) {
                    consider_overtaking_next_hallway = true;
                    // This strategy only allows to overtake 1 obstacle
                }

            }
            else if(next_second_area.type == "hallway")
            {
                next_hallway = next_second_area;
                next_hallway_task = task3;
                delta_assignment_on_overtake = 2;
                if (next_hallway.contains(obs_center_global)) {
                    consider_overtaking_next_hallway = true;
                    // This strategy only allows to overtake 1 obstacle
                }

            }
        }
    }
}


void overtakeStateMachine()
{
    obs_in_ropod_frame_pos = coordGlobalToRopod(obs_center_global, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
    v_obs_sq = current_obstacle.vel.x*current_obstacle.vel.x+current_obstacle.vel.y*current_obstacle.vel.y;
    if ((v_obs_sq < V_OBS_OVERTAKE_MAX*V_OBS_OVERTAKE_MAX) && obs_in_ropod_frame_pos.x > 0) {
        //disp("Slow obstacle, check if there is space to overtake");
        if (consider_overtaking_current_hallway) {
            rw_p_rear = getPointByID(current_hallway_task[0],pointlist);
            rw_p_front = getPointByID(current_hallway_task[1],pointlist);
            cur_obj = current_hallway;
            areaIDs = cur_obj.getPointIDs();
            ind = 0;
            qmax = areaIDs.size();
            for (int q = 0; q < qmax; ++q) {
                if (areaIDs[q].compare(rw_p_rear.id) == 0) {
                    ind = q;
                }
            }
            rotate(areaIDs.begin(), areaIDs.begin() + ind, areaIDs.end());
            lw_p_rear = getPointByID(areaIDs[3],pointlist);
            lw_p_front = getPointByID(areaIDs[2],pointlist);
        } else if (consider_overtaking_next_hallway) {
            rw_p_rear = getPointByID(next_hallway_task[0],pointlist);
            rw_p_front = getPointByID(next_hallway_task[1],pointlist);
            cur_obj = next_hallway;
            areaIDs = cur_obj.getPointIDs();
            ind = 0;
            qmax = areaIDs.size();
            for (int q = 0; q < qmax; ++q) {
                if (areaIDs[q].compare(rw_p_rear.id) == 0) {
                    ind = q;
                }
            }
            rotate(areaIDs.begin(), areaIDs.begin() + ind, areaIDs.end());
            lw_p_rear = getPointByID(areaIDs[3],pointlist);
            lw_p_front = getPointByID(areaIDs[2],pointlist);
        }

        space_left = distToSegment(obs_center_global,lw_p_rear,lw_p_front);
        space_right = distToSegment(obs_center_global,rw_p_rear,rw_p_front);
        if (current_obstacle.width > current_obstacle.depth) {
            space_left = space_left-current_obstacle.width/2;
            space_right = space_right-current_obstacle.width/2;
        } else {
            space_left = space_left-current_obstacle.depth/2;
            space_right = space_right-current_obstacle.depth/2;
        }

        if (space_right > TUBE_WIDTH_C) {
            if (j == 1) {
                ROS_INFO("No overtake necessary, passing on right should be possible");
            }
        } else if (space_right > 2*(SIZE_SIDE+ENV_TCTW_SIZE)+OBS_AVOID_MARGIN) {
            // Same state, but change tube width so ropod will
            // fit through space right
            pred_tube_width[j] = space_right;
            if (j == 1) {
                ROS_INFO("No overtake necessary, but tube size scaled down");
            }
        } else if (space_left > 2*(SIZE_SIDE+ENV_TRNS_SIZE)+ENV_TCTW_SIZE) {
            if (j == 1) {
                ROS_INFO("Can overtake on left side, there should be enough space there");
            }
            // Start overtake
            if (space_left < TUBE_WIDTH_C) {
                if (obs_in_ropod_frame_pos.x < MIN_DIST_TO_OVERTAKE && abs(obs_in_ropod_frame_pos.y) < MIN_DIST_TO_OVERTAKE) {
                    ROS_INFO("Tight overtake");
                    pred_state[j] = TIGHT_OVERTAKE;
                    //current_to_overtake_obs = to_overtake_obs;
                    if (consider_overtaking_next_hallway) {
                        u = u + delta_assignment_on_overtake; // Cesar TODO-> check if this works (instead of +2 with no consecutive hallways)!
                        update_assignment = true;
                    }
                    pred_tube_width[j] = 2*(SIZE_SIDE+ENV_TCTW_SIZE+ENV_TRNS_SIZE);
                }
            } else {
                if (obs_in_ropod_frame_pos.x < MIN_DIST_TO_OVERTAKE && abs(obs_in_ropod_frame_pos.y) < MIN_DIST_TO_OVERTAKE) {
                    ROS_INFO("Spacious overtake");
                    pred_state[j] = SPACIOUS_OVERTAKE;
                    //current_to_overtake_obs = to_overtake_obs;
                    if (consider_overtaking_next_hallway) {
                        u = u + delta_assignment_on_overtake; // Cesar TODO-> check if this works (instead of +2 with no consecutive hallways)!
                        update_assignment = true;
                    }
                    if (current_obstacle.width > current_obstacle.depth) {
                        shift_wall = space_right+current_obstacle.width+OBS_AVOID_MARGIN;
                    } else {
                        shift_wall = space_right+current_obstacle.depth+OBS_AVOID_MARGIN;
                    }
                    pred_tube_width[j] = TUBE_WIDTH_C;
                }
            }
        } else {
            if (j == 1) {
                ROS_INFO("No overtake possible, stuck behind this obstacle");
            }
        }
    }

}


/**
 * Update areas and features (corner pivot clocations, walls, etc)
 * */

void updateAreasAndFeatures()
{
    // Get old task counter
    u = pred_task_counter[j-1];
    if (j > 2) {
        uprev = pred_task_counter[j-2];
        if (u == uprev) {
            update_assignment = false;
        } else {
            update_assignment = true;
        }
    } else {
        update_assignment = true;
    }

    if (u < ka_max-1) {
        if (update_assignment) {
            area1ID = assignment[u];
            task1 = OBJ_X_TASK[u];
            area2ID = assignment[u+1];
            task2 = OBJ_X_TASK[u+1];
            if(u == (ka_max-2) )
            {
                area3ID = assignment[u+1];
                task3 = OBJ_X_TASK[u+1];
            }
            else
            {
                area3ID = assignment[u+2];
                task3 = OBJ_X_TASK[u+2];
            }

            curr_area = getAreaByID(area1ID,arealist);
            next_area = getAreaByID(area2ID,arealist);
            next_second_area = getAreaByID(area3ID,arealist);

        }

        // Bad but working statement to check if task2 contains 6 nonempty strings.
        // And if so, then it is considered as a corner assignment, otherwise as
        // go straight assignment.
        if (!task2[5].empty()) {
            if (update_assignment) {
                current_pivot = getPointByID(task2[4],pointlist);
                cur_next_hallway_rear = getPointByID(task2[2],pointlist);
                cur_next_hallway_front = getPointByID(task2[3],pointlist);
            }
            cur_pivot_local = coordGlobalToRopod(current_pivot, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
            cur_next_hallway_rear_local = coordGlobalToRopod(cur_next_hallway_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
            cur_next_hallway_front_local = coordGlobalToRopod(cur_next_hallway_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
            cur_next_hallway_angle = atan2(cur_next_hallway_front_local.y-cur_next_hallway_rear_local.y,cur_next_hallway_front_local.x-cur_next_hallway_rear_local.x);
        } else if (!task2[1].empty()) { //sum(~cellfun(@isempty,task2),2) == 2
            if (update_assignment) {
                current_inter_rear_wall = getPointByID(task2[0],pointlist);
                current_inter_front_wall = getPointByID(task2[1],pointlist);
            }
            current_inter_rear_wall_local = coordGlobalToRopod(current_inter_rear_wall, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
            current_inter_front_wall_local = coordGlobalToRopod(current_inter_front_wall, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        }
    } else {
        // Final area from assignment
        area1ID = assignment[u];
        task1 = OBJ_X_TASK[u];
        pred_state[j] = 1;  // Cruising in last HW
    }

    curr_area = getAreaByID(area1ID,arealist);
    next_area = getAreaByID(area2ID,arealist);
    next_second_area = getAreaByID(area3ID,arealist);
}


void updateStateAndTask()
{


    // Taking a turn on an intersection
    if (!task2[5].empty()) {
        //printf("Ropod entry hall task2[5] non empty: %d\n",(int)pred_ropod_on_entry_hall[j]);
        if (pred_ropod_on_entry_hall[j] && pred_state[prevstate] == CRUSING) {
            pred_state[j] = CRUSING;
            //disp([num2str(i), ': Here we can switch to the next hallway']);
            u = u+1;
            area1ID = assignment[u];
            task1 = OBJ_X_TASK[u];
            if (u < ka_max-1) {
                area2ID = assignment[u+1];
                task2 = OBJ_X_TASK[u+1];
                if(u == (ka_max-2) )
                {
                    area3ID = assignment[u+1];
                    task3 = OBJ_X_TASK[u+1];
                }
                else
                {
                    area3ID = assignment[u+2];
                    task3 = OBJ_X_TASK[u+2];
                }
            }
            else
            {
                area1ID = assignment[ka_max-1];
                task1 = OBJ_X_TASK[ka_max-1];
                area2ID = assignment[ka_max-1];
                task2 = OBJ_X_TASK[ka_max-1];
                area3ID = assignment[ka_max-1];
                task3 = OBJ_X_TASK[ka_max-1];
            }
            curr_area = getAreaByID(area1ID,arealist);
            next_area = getAreaByID(area2ID,arealist);
            next_second_area = getAreaByID(area3ID,arealist);


        } else if (pred_ropod_on_entry_inter[j] && pred_state[prevstate] == CRUSING) {
            // If cruising and the y position of the ropod exceeds the y
            // position of the entry
            if(j==1)printf("Entry detected to turn intersection u = %d\n",u);
            pred_state[j] = ENTRY_BEFORE_TURN_ON_INTERSECTION;

        } else if (cur_pivot_local.x < SIZE_FRONT_ROPOD  && pred_state[prevstate] == ENTRY_BEFORE_TURN_ON_INTERSECTION) {
            // If in entry and the y position of the ropod exceeds the y
            // position of the intersection
            pred_state[j] = ACCELERATE_ON_INTERSECTION;

        } else if (cur_pivot_local.x <= SIZE_FRONT_RAX + START_STEERING_EARLY && pred_state[prevstate] == ACCELERATE_ON_INTERSECTION) {
            // If middle of vehicle is on y height of pivot
            pred_state[j] = ALIGN_AXIS_AT_INTERSECTION;

        } else if (cur_pivot_local.x <= -ROPOD_TO_AX+START_STEERING_EARLY && pred_state[prevstate] == ALIGN_AXIS_AT_INTERSECTION) {
            // If rearaxle is aligned with the pivot minus sse
            if(j==1)printf("Turning on  u = %d\n",u+1);
            pred_state[j] = TURNING;

        } else if (-ROTATED_ENOUGH_TRES < cur_next_hallway_angle && cur_next_hallway_angle < ROTATED_ENOUGH_TRES && pred_state[prevstate] == TURNING) {
            // If ropod has turned enough

            pred_state[j] = CRUSING;
            //disp([num2str(i), ': Here we can switch to the next task']);
            u = u+2;
            if(j==1)printf("Done with turning intersection, now on u = %d\n",u);
            area1ID = assignment[u];
            task1 = OBJ_X_TASK[u];
            if (u < ka_max-1)
            {
                area2ID = assignment[u+1];
                task2 = OBJ_X_TASK[u+1];
                if(u == (ka_max-2) )
                {
                    area3ID = assignment[u+1];
                    task3 = OBJ_X_TASK[u+1];
                }
                else
                {
                    area3ID = assignment[u+2];
                    task3 = OBJ_X_TASK[u+2];
                }
            }
            else
            {
                area1ID = assignment[ka_max-1];
                task1 = OBJ_X_TASK[ka_max-1];
                area2ID = assignment[ka_max-1];
                task2 = OBJ_X_TASK[ka_max-1];
                area3ID = assignment[ka_max-1];
                task3 = OBJ_X_TASK[ka_max-1];
            }
            curr_area = getAreaByID(area1ID,arealist);
            next_area = getAreaByID(area2ID,arealist);
            next_second_area = getAreaByID(area3ID,arealist);

        } else {

            pred_state[j] = pred_state[prevstate];
            update_state_points = false;
        }
    // Going straight on an intersection / ot between hallways
    } else if (!task2[1].empty()) {
        if (pred_ropod_on_entry_hall[j] && pred_state[prevstate] == CRUSING) {
            pred_state[j] = CRUSING;
            //disp([num2str(i), ': Here we can switch to the next hallway']);
            u = u+1;
            if(j==1)printf("Entry detected between halls u = %d\n",u);
            if (u < ka_max-1)
            {
                area1ID = assignment[u];
                task1 = OBJ_X_TASK[u];
                area2ID = assignment[u+1];
                task2 = OBJ_X_TASK[u+1];
                if(u == (ka_max-2) )
                {
                    area3ID = assignment[u+1];
                    task3 = OBJ_X_TASK[u+1];
                }
                else
                {
                    area3ID = assignment[u+2];
                    task3 = OBJ_X_TASK[u+2];
                }
            }
            else
            {
                area1ID = assignment[ka_max-1];
                task1 = OBJ_X_TASK[ka_max-1];
                area2ID = assignment[ka_max-1];
                task2 = OBJ_X_TASK[ka_max-1];
                area3ID = assignment[ka_max-1];
                task3 = OBJ_X_TASK[ka_max-1];
            }
            curr_area = getAreaByID(area1ID,arealist);
            next_area = getAreaByID(area2ID,arealist);
            next_second_area = getAreaByID(area3ID,arealist);


        }else if (pred_ropod_on_entry_inter[j] && pred_state[prevstate] == CRUSING) {
            // If cruising and the y position of the ropod exceeds the y
            // position of the entry
            if(j==1)printf("Entry detected to straight intersection u = %d\n",u);
            pred_state[j] = ENTRY_BEFORE_GOING_STRAIGHT_ON_INTERSECTION;
        } else if (current_inter_rear_wall_local.x < SIZE_FRONT_ROPOD+START_STEERING_EARLY && pred_state[prevstate] == ENTRY_BEFORE_GOING_STRAIGHT_ON_INTERSECTION) {
            // If in entry and the y position of the ropod exceeds the y
            // position of the intersection
            if(j==1)printf("Staright on u = %d\n",u+1);
            pred_state[j] = GOING_STRAIGHT_ON_INTERSECTION;
        } else if (current_inter_front_wall_local.x < -D_AX/2 && pred_state[prevstate] == GOING_STRAIGHT_ON_INTERSECTION) {

            pred_state[j] = CRUSING;
            u = u+2;
            if(j==1)printf("Done with straight intersection, now on u = %d\n",u);
            if (u < ka_max-1) {
                area1ID = assignment[u];
                task1 = OBJ_X_TASK[u];
                area2ID = assignment[u+1];
                task2 = OBJ_X_TASK[u+1];
                if(u == (ka_max-2) )
                {
                    area3ID = assignment[u+1];
                    task3 = OBJ_X_TASK[u+1];
                }
                else
                {
                    area3ID = assignment[u+2];
                    task3 = OBJ_X_TASK[u+2];
                }
            }
            else
            {
                area1ID = assignment[ka_max-1];
                task1 = OBJ_X_TASK[ka_max-1];
                area2ID = assignment[ka_max-1];
                task2 = OBJ_X_TASK[ka_max-1];
                area3ID = assignment[ka_max-1];
                task3 = OBJ_X_TASK[ka_max-1];
            }
            curr_area = getAreaByID(area1ID,arealist);
            next_area = getAreaByID(area2ID,arealist);
            next_second_area = getAreaByID(area3ID,arealist);


        } else {
            pred_state[j] = pred_state[prevstate];
            update_state_points = false;
        }
    }

    if(u >= ka_max)
    {
        u = ka_max-1;
    }

    if (pred_state[prevstate] == TIGHT_OVERTAKE || pred_state[prevstate] == SPACIOUS_OVERTAKE) {
        if (no_obs > 0) {
            current_obs_in_ropod_frame_pos = coordGlobalToRopod(obs_center_global, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
            //disp(['Obs is ',num2str(obs_in_ropod_frame_pos.x), ' m in front of ropod']);
            if (current_obs_in_ropod_frame_pos.x+current_obstacle.depth/2+D_AX+SIZE_REAR < 0) {
                pred_state[j] = CRUSING;
                update_state_points = true;
                pred_tube_width[j] = TUBE_WIDTH_C;
            } else {
                pred_state[j] = pred_state[prevstate];
                update_state_points = false;
            }
        } else {
            // Ropod doesn't see obs anymore, return cruising.
            // When obt disappears depends on the setting how long it takes before obs disappears after not seeing it (entity-timeout)
            // It can be found in /catkin_workspace/src/applications/ropod_navigation_test/config/model-example-ropod-navigation-ED.yaml
            pred_state[j] = CRUSING;
            update_state_points = true;
            pred_tube_width[j] = TUBE_WIDTH_C;
        }
    }

    // TODO: This code was moved. It was right before steerings were computed based on prediction. Check if still works on al cases
    // Exception for first plan
    if (j == 1) {
        update_state_points = true;
    }
    // Exception for when current and previous state are turning
    // And the state before that is aligning, as the state jump
    // happens immediately if that is required and then the state
    // points need to be updated.
    if (j > 1) {
        if (pred_state[j] == TURNING && pred_state[j-1] == TURNING && (pred_state[j-2] == ACCELERATE_ON_INTERSECTION || pred_state[j-2] == ALIGN_AXIS_AT_INTERSECTION)) {
            update_state_points = true;
        }
    }

    pred_task_counter[j] = u;

    // Monitor if we don't bump into front wall when aligning
    // rearaxle to pivot. If so, switch to turn state (5)
    if (pred_state[j] == ACCELERATE_ON_INTERSECTION || pred_state[j] == ALIGN_AXIS_AT_INTERSECTION) {
        if (update_state_points) {
            wall_front_p0 = getPointByID(task2[0],pointlist);
            wall_front_p1 = getPointByID(task2[1],pointlist);
            //global_wall_front_p0 = [wall_front_p0.x, wall_front_p0.y];
            //global_wall_front_p1 = [wall_front_p1.x, wall_front_p1.y];
        }

        local_wall_front_p0 = coordGlobalToRopod(wall_front_p0, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        local_wall_front_p1 = coordGlobalToRopod(wall_front_p1, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        if (do_lines_intersect(local_front_ropod_dilated_p0, local_front_ropod_dilated_p1, local_wall_front_p0, local_wall_front_p1)) {
            pred_state[j] = TURNING;
            update_state_points = true;
            //disp("Switched early while aligning pivot because too close to front wall");
        }
    }

}

ros::Publisher mapmarker_pub;
ros::Publisher wallmarker_pub;
/**
 * Compute steering and default forward acceleration based on computed state and local features
 * */
void computeSteeringAndVelocity()
{
    // Perform the appropriate action according to finite state machine
    if (pred_state[j] == CRUSING || pred_state[j] == GOING_STRAIGHT_ON_INTERSECTION || pred_state[j] == TIGHT_OVERTAKE || pred_state[j] == SPACIOUS_OVERTAKE) {
        // disp([num2str[j],' - Ropod is now cruising']);
        if (pred_state[j] == CRUSING) {   // Cruising up
            if (update_state_points) {
                point_rear = getPointByID(task1[0],pointlist);
                point_front = getPointByID(task1[1],pointlist);
                //if(j==1 && u == ka_max-1) ROS_INFO("Cruising Point rear: %s, Point front: %s", point_rear.id.c_str(), point_front.id.c_str());
                glob_wallpoint_front.x = point_front.x;
                glob_wallpoint_front.y = point_front.y;
                glob_wallpoint_rear.x = point_rear.x;
                glob_wallpoint_rear.y = point_rear.y;
            }
            v_des = V_CRUISING;
        } else if (pred_state[j] == GOING_STRAIGHT_ON_INTERSECTION) { // Straight on inter
            if (update_state_points) {
                point_rear = getPointByID(task2[0],pointlist);
                point_front = getPointByID(task2[1],pointlist);
                glob_wallpoint_front.x = point_front.x;
                glob_wallpoint_front.y = point_front.y;
                glob_wallpoint_rear.x = point_rear.x;
                glob_wallpoint_rear.y = point_rear.y;
            }
            v_des = V_INTER_ACC;
        } else if (pred_state[j] == TIGHT_OVERTAKE) { // Tight overtake
            rw_p_rear = getPointByID(task1[0],pointlist);
            cur_obj = getAreaByID(area1ID,arealist);
            areaIDs = cur_obj.getPointIDs();
            ind = 0;
            qmax = areaIDs.size();
            for (int q = 0; q < qmax; ++q) {
                if (areaIDs[q].compare(rw_p_rear.id) == 0) {
                    ind = q;
                }
            }
            rotate(areaIDs.begin(), areaIDs.begin() + ind, areaIDs.end());
            lw_p_rear = getPointByID(areaIDs[3],pointlist);
            lw_p_front = getPointByID(areaIDs[2],pointlist);
            wallang = atan2(lw_p_front.y-lw_p_rear.y,lw_p_front.x-lw_p_rear.x);
            //glob_wallpoint_front = [lw_front.x, lw_front.y]+pred_tube_width[j]*[cos(wallang-M_PI/2), sin(wallang-M_PI/2)];
            glob_wallpoint_front.x = lw_p_front.x+pred_tube_width[j]*cos(wallang-M_PI/2);
            glob_wallpoint_front.y = lw_p_front.y+pred_tube_width[j]*sin(wallang-M_PI/2);
            //glob_wallpoint_rear = [lw_rear.x, lw_rear.y]+pred_tube_width[j]*[cos(wallang-M_PI/2), sin(wallang-M_PI/2)];
            glob_wallpoint_rear.x = lw_p_rear.x+pred_tube_width[j]*cos(wallang-M_PI/2);
            glob_wallpoint_rear.y = lw_p_rear.y+pred_tube_width[j]*sin(wallang-M_PI/2);
            v_des = V_OVERTAKE;
        } else if (pred_state[j] == SPACIOUS_OVERTAKE) { // Spacious overtake
            point_rear = getPointByID(task1[0],pointlist);
            point_front = getPointByID(task1[1],pointlist);
            wallang = atan2(point_front.y-point_rear.y,point_front.x-point_rear.x);
            //glob_wallpoint_front = [point_front.x, point_front.y]+shift_wall*[cos(wallang+M_PI/2), sin(wallang+M_PI/2)];
            //glob_wallpoint_rear = [point_rear.x, point_rear.y]+shift_wall*[cos(wallang+M_PI/2), sin(wallang+M_PI/2)];
            glob_wallpoint_front.x = point_front.x+shift_wall*cos(wallang+M_PI/2);
            glob_wallpoint_front.y = point_front.y+shift_wall*sin(wallang+M_PI/2);
            glob_wallpoint_rear.x = point_rear.x+shift_wall*cos(wallang+M_PI/2);
            glob_wallpoint_rear.y = point_rear.y+shift_wall*sin(wallang+M_PI/2);
            v_des = V_OVERTAKE;
        }
        local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        showWallPoints(local_wallpoint_front, local_wallpoint_rear, wallmarker_pub);

        pred_phi_des[j] = getSteering(local_wallpoint_front, local_wallpoint_rear, pred_tube_width[j]);
    } else if (pred_state[j] == ENTRY_BEFORE_TURN_ON_INTERSECTION || pred_state[j] == ENTRY_BEFORE_GOING_STRAIGHT_ON_INTERSECTION) {
        // disp([num2str[j],' - Ropod is now in entry']);
        if (update_state_points) {
            point_rear = getPointByID(task1[0],pointlist);
            point_front = getPointByID(task1[1],pointlist);
            glob_wallpoint_front.x = point_front.x;
            glob_wallpoint_front.y = point_front.y;
            glob_wallpoint_rear.x = point_rear.x;
            glob_wallpoint_rear.y = point_rear.y;
        }
        local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        showWallPoints(local_wallpoint_front, local_wallpoint_rear, wallmarker_pub);
        pred_phi_des[j] = getSteering(local_wallpoint_front, local_wallpoint_rear, pred_tube_width[j]);
        v_des = V_ENTRY;
    } else if (pred_state[j] == ACCELERATE_ON_INTERSECTION) {
        // disp([num2str[j],' - Ropod is at inter, driving forward']);
        if (update_state_points) {
            point_rear = getPointByID(task1[0],pointlist);
            point_front = getPointByID(task1[1],pointlist);
            glob_wallpoint_front.x = point_front.x;
            glob_wallpoint_front.y = point_front.y;
            glob_wallpoint_rear.x = point_rear.x;
            glob_wallpoint_rear.y = point_rear.y;
        }

        local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        showWallPoints(local_wallpoint_front, local_wallpoint_rear, wallmarker_pub);
        pred_phi_des[j] = getSteering(local_wallpoint_front, local_wallpoint_rear, pred_tube_width[j]);
        v_des = V_INTER_ACC;

        // Monitor if we don't bump into front wall
        if (update_state_points) {
            wall_front_p0 = getPointByID(task2[0],pointlist);
            wall_front_p1 = getPointByID(task2[1],pointlist);

            global_wall_front_p0.x = wall_front_p0.x;
            global_wall_front_p0.y = wall_front_p0.y;
            global_wall_front_p1.x = wall_front_p1.x;
            global_wall_front_p1.y = wall_front_p1.y;
        }

        local_wall_front_p0 = coordGlobalToRopod(global_wall_front_p0, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        local_wall_front_p1 = coordGlobalToRopod(global_wall_front_p1, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        if (do_lines_intersect(local_front_ropod_dilated_p0, local_front_ropod_dilated_p1, local_wall_front_p0, local_wall_front_p1)) {
            pred_state[j] = TURNING;
            update_state_points = true;
            //disp("Switched early while aligning pivot because too close to front wall");
        }
    } else if (pred_state[j] == ALIGN_AXIS_AT_INTERSECTION) {
        // disp([num2str[j],' - Ropod is at inter, driving forward']);
        if (update_state_points) {
            point_rear = getPointByID(task1[0],pointlist);
            point_front = getPointByID(task1[1],pointlist);
            glob_wallpoint_front.x = point_front.x;
            glob_wallpoint_front.y = point_front.y;
            glob_wallpoint_rear.x = point_rear.x;
            glob_wallpoint_rear.y = point_rear.y;
        }

        local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        showWallPoints(local_wallpoint_front, local_wallpoint_rear, wallmarker_pub);
        pred_phi_des[j] = getSteering(local_wallpoint_front, local_wallpoint_rear, pred_tube_width[j]);
        v_des = V_INTER_DEC;

        // Monitor if we don't bump into front wall
        if (update_state_points) {
            wall_front_p0 = getPointByID(task2[0],pointlist);
            wall_front_p1 = getPointByID(task2[1],pointlist);
            global_wall_front_p0.x = wall_front_p0.x;
            global_wall_front_p0.y = wall_front_p0.y;
            global_wall_front_p1.x = wall_front_p1.x;
            global_wall_front_p1.y = wall_front_p1.y;
        }

        local_wall_front_p0 = coordGlobalToRopod(global_wall_front_p0, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        local_wall_front_p1 = coordGlobalToRopod(global_wall_front_p1, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        if (do_lines_intersect(local_front_ropod_dilated_p0, local_front_ropod_dilated_p1, local_wall_front_p0, local_wall_front_p1)) {
            pred_state[j] = TURNING;
            update_state_points = true;
            //disp("Switched early while aligning pivot because too close to front wall");
        }
    } else if (pred_state[j] == TURNING) {
        // disp([num2str[j],' - Ropod is at inter, taking the turn']);
        if (update_state_points) {
            point_rear = getPointByID(task2[0],pointlist);
            point_front = getPointByID(task2[1],pointlist);
            point_pivot = getPointByID(task2[4],pointlist);
            glob_wallpoint_front.x = point_front.x;
            glob_wallpoint_front.y = point_front.y;
            glob_wallpoint_rear.x = point_rear.x;
            glob_wallpoint_rear.y = point_rear.y;
        }

        dir_cw = true;
        if (task2[5].compare("left") == 0) {
        //if (strcmp(task2{6},'left')) {
            dir_cw = false; // direction 0 = CCW, 1 = CW
        }

        local_pivot = coordGlobalToRopod(point_pivot, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        if (!sharp_corner[u+1]) {
            local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
            local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
            //pred_phi_des[j] = getSteeringTurn(ropod_length, size_side, feeler_size_steering, d_ax, dir_cw, local_pivot, local_wallpoint_front, local_wallpoint_rear,  follow_wall_distance, env_tctw_size, env_trns_size_cornering, env_carrot_size);
            pred_phi_des[j] = getSteeringTurn(local_pivot, dir_cw, local_wallpoint_front, local_wallpoint_rear);
        } else {
            //pred_phi_des[j] = getSteeringTurnSharp(pred_x_ropod(j-1), pred_y_ropod(j-1), pred_plan_theta[j-1], size_front_ropod, size_side, feeler_size_steering, d_ax, dir_cw, task2, pointlist, follow_wall_distance, env_tctw_size, env_trns_size_cornering, env_carrot_size);
            pred_phi_des[j] = getSteeringTurnSharp(pred_xy_ropod[j-1], pred_plan_theta[j-1], dir_cw, task2, pointlist);
        }

        v_des = V_INTER_TURNING;
    }

    // Wrap to [-pi,pi] domain
    pred_phi_des[j] = wrapToPi(pred_phi_des[j]);

    // Saturate steering rate
    if (abs(pred_phi_des[j]-prev_pred_phi_des) > DELTA_DOT_LIMIT/(double)F_PLANNER) {
        //disp("Delta steering too large, steering saturated");
        pred_phi_des[j] = prev_pred_phi_des + sgn(pred_phi_des[j]-prev_pred_phi_des)*DELTA_DOT_LIMIT/(double)F_PLANNER;
        // Decrease vel leads to better corners
        // The velocity is already decreased in the state machine, but this is just a harsh backup
        // pred_steer_rate_saturation[j] = 1;
        if (v_des > V_STEERSATURATION) {
            v_des = V_STEERSATURATION;
        }
        //disp(['In saturation - j: ', num2str(j) ,', Phides: ', num2str(pred_phi_des(j)), ' // Prev phides: ' , num2str(prev_pred_phi_des), ', v_des = ', num2str(v_des)]);
    }
    prev_pred_phi_des = pred_phi_des[j];

    // Applying the v_scale (scales down vel if collision is detected in previous prediction and therefore it failed)
    // And then we calculate the acceleration for the predictions.
    v_des_scaled[j] = v_des*v_scale;
    vel_dif = abs(pred_v_ropod_plan[j-1]-v_des_scaled[j]);   // Difference between actual and desired velocity
    if (vel_dif < max_delta_v) {
        v_new = v_des_scaled[j];
        pred_accel[j] = (v_des_scaled[j]-pred_v_ropod_plan[j-1])/delta_t;
    } else { // Adapt velocity with maximum acceleration
        v_new = pred_v_ropod_plan[j-1]+sgn(v_des_scaled[j]-pred_v_ropod_plan[j-1])*max_delta_v;
        pred_accel[j] = sgn(v_des_scaled[j]-pred_v_ropod_plan[j-1])*A_MAX;
    }
    //ROS_INFO("pred_accel: %f", pred_accel[j]);
}


/**
 * Simulate robot during current prediction step.
 * Simulations are stacked over the different prediction steps.
 * */
void simulateRobotDuringCurrentPredictionStep()
{
    m_prev = m;
    t_pred_prev = t_pred[m];
    t_pred_j[j] = t_pred_prev+F_FSTR*TS;

    // Simulate ropod motion with current plan (Simulation is done faster than controller sample time)
    for (int q = 1; q <= F_FSTR; ++q) { // q = [1, 2, ..., F_FSTR]
        m = m_prev+q;           // Current iteration
        t_pred[m] = t_pred_prev+q*TS;

        pred_phi[m] = (1-lpf)*pred_phi[m-1]+lpf*pred_phi_des[j];
        pred_theta[m] = wrapToPi(pred_theta[m-1]+pred_thetadot[m-1]*TS);
        pred_v_ropod[m] = pred_v_ropod[m-1]+pred_accel[j]*TS;

        pred_xdot[m] = pred_v_ropod[m]*cos(pred_phi[m])*cos(pred_theta[m]);
        pred_ydot[m] = pred_v_ropod[m]*cos(pred_phi[m])*sin(pred_theta[m]);
        pred_thetadot[m] = pred_v_ropod[m]*1/D_AX*sin(pred_phi[m]);

        pred_x_rearax[m] = pred_x_rearax[m-1]+pred_xdot[m]*TS;
        pred_y_rearax[m] = pred_y_rearax[m-1]+pred_ydot[m]*TS;
    }
}

/**
 * Check for collision
 * Either for obstacles or virtual walls(to be added)
 * */
void checkForCollisions()
{
    if (no_obs > 0) {
        pred_x_obs[j] = pred_x_obs[j-1]+current_obstacle.vel.x*TS*F_FSTR;
        pred_y_obs[j] = pred_y_obs[j-1]+current_obstacle.vel.y*TS*F_FSTR;
    }

    // Dilated vehicle
    pred_ropod_dil_rb.x = pred_x_rearax[m] + (SIZE_REAR+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]+M_PI) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]-M_PI/2);
    pred_ropod_dil_rb.y = pred_y_rearax[m] + (SIZE_REAR+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]+M_PI) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]-M_PI/2);
    pred_ropod_dil_lb.x = pred_x_rearax[m] + (SIZE_REAR+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]+M_PI) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]+M_PI/2);
    pred_ropod_dil_lb.y = pred_y_rearax[m] + (SIZE_REAR+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]+M_PI) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]+M_PI/2);
    pred_ropod_dil_lt.x = pred_x_rearax[m] + (SIZE_FRONT_RAX+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]+M_PI/2);
    pred_ropod_dil_lt.y = pred_y_rearax[m] + (SIZE_FRONT_RAX+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]+M_PI/2);
    pred_ropod_dil_rt.x = pred_x_rearax[m] + (SIZE_FRONT_RAX+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]-M_PI/2);
    pred_ropod_dil_rt.y = pred_y_rearax[m] + (SIZE_FRONT_RAX+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]-M_PI/2);

    ropod_colliding_obs = false;
    pred_ropod_colliding_obs[j] = false;
    // Obstacle detection (crappy implementation in C++)
    if (t_pred[m] < T_PRED_OBS_COLLISION && no_obs > 0) {
        obslt.x = pred_x_obs[j]+current_obstacle.width/2*cos(obs_theta)-current_obstacle.depth/2*sin(obs_theta);
        obslt.y = pred_y_obs[j]+current_obstacle.width/2*sin(obs_theta)+current_obstacle.depth/2*cos(obs_theta);
        obsrt.x = pred_x_obs[j]+current_obstacle.width/2*cos(obs_theta)+current_obstacle.depth/2*sin(obs_theta);
        obsrt.y = pred_y_obs[j]+current_obstacle.width/2*sin(obs_theta)-current_obstacle.depth/2*cos(obs_theta);
        obslb.x = pred_x_obs[j]-current_obstacle.width/2*cos(obs_theta)-current_obstacle.depth/2*sin(obs_theta);
        obslb.y = pred_y_obs[j]-current_obstacle.width/2*sin(obs_theta)+current_obstacle.depth/2*cos(obs_theta);
        obsrb.x = pred_x_obs[j]-current_obstacle.width/2*cos(obs_theta)+current_obstacle.depth/2*sin(obs_theta);
        obsrb.y = pred_y_obs[j]-current_obstacle.width/2*sin(obs_theta)-current_obstacle.depth/2*cos(obs_theta);
        pred_ropod_colliding_obs[j] = do_shapes_overlap(pred_ropod_dil_rb, pred_ropod_dil_lb, pred_ropod_dil_lt, pred_ropod_dil_rt, obsrt, obslt, obslb, obsrb);
        ropod_colliding_obs = pred_ropod_colliding_obs[j];
    }

    // TODO: Add here also check with raw laser (for instance of non-associated objects) data or costmap
    if (t_pred[m] < T_PRED_OBS_COLLISION ) // TODO: Condition should be smarter. For instance based on time/distance to stop? . Also the robot should stop at a minimum distance in front. for now we based it on time
    {
        // Here check for collision with laser data
        // Create Area and use contain method with the laser data.
        for(unsigned int iScan = 0; iScan < laser_meas_points.size(); iScan++)
        {
            AreaQuad robot_footprint(pred_ropod_dil_rb, pred_ropod_dil_lb, pred_ropod_dil_lt, pred_ropod_dil_rt);
            Point laser_point(laser_meas_points[iScan].x, laser_meas_points[iScan].y);
            pred_ropod_colliding_obs[j] = robot_footprint.contains(laser_point);
            ropod_colliding_obs = pred_ropod_colliding_obs[j];
            if(ropod_colliding_obs)
                break;
        }
    }


    // Predict intersection times (left out for now)
    ropod_colliding_wall = false;

    // TODO: Finish next line to add checking collision with walls!

    // Predict intersection with walls
    // if ((u < ka_max-1) && update_assignment) {
    //     walls = getWalls(assignment[u],assignment[u+1],assignment[u+2],arealist);
    // }
    // if (t_pred[m] < T_PRED_WALL_COLLISION) {
    //     pred_ropod_wall_collision(j) = 0;
    //     for wallidx = 1:size(walls,1)
    //         wall = walls(wallidx,:);
    //         wp0 = getPointByID(wall{1},pointlist);
    //         wp1 = getPointByID(wall{2},pointlist);
    //         ropod = [pred_ropod_rb(j,:); pred_ropod_lb(j,:); pred_ropod_lt(j,:); pred_ropod_rt(j,:)];
    //         if does_line_intersect_shape(wp0, wp1, ropod)
    //             pred_ropod_wall_collision(j) = 1;
    //             // disp([num2str(t_pred(m)),': Oh noes I collided wiv a wall']);
    //         end
    //     end
    // }
}

ros::Publisher ropodmarker_pub;
void visualizeRopodMarkers()
{
    vis_points.id = 1;
    vis_points.color.r = 0.0;
    vis_points.color.g = 0.0;
    vis_points.color.b = 0.0;
    vis_points.scale.x = 0.1;
    vis_points.scale.y = 0.1;
    geometry_msgs::Point vis_p;
    vis_p.x = obsrt.x; vis_p.y = obsrt.y; vis_points.points.push_back(vis_p);
    vis_p.x = obslt.x; vis_p.y = obslt.y; vis_points.points.push_back(vis_p);
    vis_p.x = obslb.x; vis_p.y = obslb.y; vis_points.points.push_back(vis_p);
    vis_p.x = obsrb.x; vis_p.y = obsrb.y; vis_points.points.push_back(vis_p);
    ropodmarker_pub.publish(vis_points);
}

// route planner integration
class NapoleonPlanner
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ropod_ros_msgs::GoToAction> as_;
    actionlib::SimpleActionClient<ropod_ros_msgs::RoutePlannerAction> ac_;
    std::string action_name_;
    ropod_ros_msgs::GoToFeedback feedback_;
    ropod_ros_msgs::GoToResult result_;
    ropod_ros_msgs::RoutePlannerResult route_planner_result_;
    bool status_;

public:
    NapoleonPlanner(std::string name) :
    as_(nh_, name, boost::bind(&NapoleonPlanner::executeCB, this, _1), false),
    action_name_(name), ac_("/route_planner", true), status_(false)
    {
        ROS_INFO("Waiting for route planner action server to start");
        ac_.waitForServer();
        ROS_INFO("Connected to route planner action server");
        // waiting for route planner action server to start
        as_.start();
        ROS_INFO("Waiting for GOTO action");
    }

    ~NapoleonPlanner(void)
    {
    }

    bool getStatus()
    {
        return status_;
    }

    ropod_ros_msgs::RoutePlannerResult getPlannerResult()
    {
        return route_planner_result_;
    }


    void plannerResultCB(const actionlib::SimpleClientGoalState& state, const ropod_ros_msgs::RoutePlannerResultConstPtr& result)
    {
        route_planner_result_ = *result;
    }

    void executeCB(const ropod_ros_msgs::GoToGoalConstPtr &goal)
    {
        std::vector<ropod_ros_msgs::Area> area_list = goal->action.areas;
        ropod_ros_msgs::RoutePlannerGoal route_planner_goal;
        route_planner_goal.areas = area_list;
        ac_.sendGoal(route_planner_goal, boost::bind(&NapoleonPlanner::plannerResultCB, this, _1, _2));

        //wait for the action to return
        bool finished_before_timeout = ac_.waitForResult(ros::Duration(60.0));

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
            status_ = true;
        }
        else
        {
            ROS_INFO("Action did not finish before the time out.");
            status_ = false;
        }
    }
};
ropod_ros_msgs::RoutePlannerResult debug_route_planner_result_;

void getDebugRoutePlanCallback(const ropod_ros_msgs::RoutePlannerResultConstPtr& result)
{
    debug_route_planner_result_ = *result;

    ROS_INFO("new debug plan received");
    start_navigation = true;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "route_navigation");
    ros::NodeHandle nroshndl("~");
    ros::Rate rate(F_PLANNER);

    ros::Subscriber goal_cmd_sub = nroshndl.subscribe<geometry_msgs::PoseStamped>("/route_navigation/simple_goal", 10, simpleGoalCallback);
    ros::Subscriber amcl_pose_sub = nroshndl.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, getAmclPoseCallback);
    ros::Subscriber ropod_odom_sub = nroshndl.subscribe<nav_msgs::Odometry>("/ropod/odom", 100, getOdomVelCallback);
    ros::Subscriber ropod_debug_plan_sub = nroshndl.subscribe< ropod_ros_msgs::RoutePlannerResult >("/ropod/debug_route_plan", 1, getDebugRoutePlanCallback);

//    ros::Subscriber obstacle_sub = nroshndl.subscribe<ed_gui_server::objsPosVel>("/ed/gui/objectPosVel", 10, getObstaclesCallback);
    ros::Publisher vel_pub = nroshndl.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    // Subscribe to topic with non-associated laser points (for now all laser points)
    std::string laser_topic("/projected_scan_front");
    //std::string laser_topic("/ropod/laser/scan");
    unsigned int bufferSize = 1;
    ros::Subscriber scan_sub = nroshndl.subscribe<sensor_msgs::LaserScan>(laser_topic, bufferSize, scanCallback);
    // Visualize map nodes and robot
    ropodmarker_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_driving/ropodpoints", 1);
    mapmarker_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_driving/vmnodes", 100, true);
    wallmarker_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_driving/right_side_wall", 10, true);
    tf_listener_ = new tf::TransformListener;


    NapoleonPlanner napoleon_planner_("/ropod/goto");


    while(ros::ok())
    {
        if(napoleon_planner_.getStatus())
        {
            break;
        }
        ros::spinOnce();
    }

    /*
    ROS_INFO("Wait for debug plan on topic");
    while(ros::ok())
    {
        if(start_navigation)
        {
            break;
        }
        ros::spinOnce();
    }
    */

    ROS_INFO("Now preparing the plan");
    initializeVisualizationMarkers();
    //std::vector<ropod_ros_msgs::Area> planner_areas = napoleon_planner_.getPlannerResult().areas;
    std::vector<ropod_ros_msgs::Area> planner_areas = debug_route_planner_result_.areas;
    int intermediate_area_id_counter = 10000;
    for (int i = 0; i < planner_areas.size(); i++)
    {
        for (int j = 0; j < planner_areas[i].sub_areas.size(); j++)
        {
            for (int k = 0; k < planner_areas[i].sub_areas[j].geometry.vertices.size(); k++)
            {
                pointlist.push_back(PointID(planner_areas[i].sub_areas[j].geometry.vertices[k].x,
                                             planner_areas[i].sub_areas[j].geometry.vertices[k].y,
                                             std::to_string(planner_areas[i].sub_areas[j].geometry.vertices[k].id)));
            }

            int points_size = pointlist.size();
            std::string sub_area_type = planner_areas[i].type;

            if (sub_area_type == "junction")
            {
                sub_area_type = "inter";
            }
            else
            {
                sub_area_type = "hallway";
            }

            int sub_area_id;
            if (points_size >= 4)
            {

                if(planner_areas[i].sub_areas[j].id != "")
                {
                    sub_area_id = std::stoi(planner_areas[i].sub_areas[j].id.c_str());
                }
                else
                {
                    sub_area_id = intermediate_area_id_counter;
                    intermediate_area_id_counter = intermediate_area_id_counter + 1;
                }

                arealist.push_back(AreaQuadID(pointlist[points_size-4],
                                               pointlist[points_size-3],
                                               pointlist[points_size-2],
                                               pointlist[points_size-1],
                                               sub_area_id,
                                               sub_area_type));

                // Plan visualization
                vis_plan.points.clear();
                vis_plan.header.stamp = ros::Time::now();
                vis_plan.id = 10*i+j;
                geometry_msgs::Point p;
                p.x = pointlist[points_size-4].x;
                p.y = pointlist[points_size-4].y;
                vis_plan.points.push_back(p);
                p.x = pointlist[points_size-3].x;
                p.y = pointlist[points_size-3].y;
                vis_plan.points.push_back(p);
                p.x = pointlist[points_size-2].x;
                p.y = pointlist[points_size-2].y;
                vis_plan.points.push_back(p);
                p.x = pointlist[points_size-1].x;
                p.y = pointlist[points_size-1].y;
                vis_plan.points.push_back(p);
                p.x = pointlist[points_size-4].x;
                p.y = pointlist[points_size-4].y;
                vis_plan.points.push_back(p);
                mapmarker_pub.publish(vis_plan);
            }
            else
            {
                ROS_ERROR("AREA WITH LESS THAN 4 POINTS");
            }
            ROS_INFO("Sub area id: %d | Sub area type: %s", sub_area_id, sub_area_type.c_str());
            assignment.push_back(sub_area_id);
        }
    }

    ROS_INFO("Now starting navigation");
    // start_navigation = true;


    // update area value
    ka_max = assignment.size();  // Assignment length

    cur_obj = getAreaByID(assignment[0],arealist);


    initializeAssignment();

    ROS_INFO("Wait for 2D Nav Goal to start (goal can be anywhere, doesn't influence program)");


    std::ofstream myfile;
    //myfile.open ("/simdata/ropod_" + get_date() +".txt");
    myfile.open ("/home/cesar/Documents/simdata/ropod_" + get_date() +".txt");
    myfile << "time" << "\t" << "tictoc" << "\t" << "state" << "\t" << "task counter" << "\t" << "tube width" << "\t" <<
            "phi des" << "\t" << "v ropod odom" << "\t"<< "v ropod cmd" << "\t" << "x ropod" << "\t" << "y ropod" << "\t" <<
            "theta ropod" << "\t" "x obs" << "\t" << "y obs" << "\t" << "theta obs" << "\t" << "obs width" << "\t" <<
            "obs depth" << "\t" << "obs vx" << "\t" << "obs vy" << "\t" << "des accel" <<"\n";

    std::clock_t start_loop;


    while(nroshndl.ok() && !ropod_reached_target)
    {
        // Process scan data
        if(scan_available)
        {
            getLatestScanData();
            scan_available = false;
        }


        if (start_navigation)
        {

        start_loop = std::clock();

        ropod_colliding_obs = true;     // Initially set to true
        ropod_colliding_wall = true;    // Initially set to true
        // bool predict_intersection_time = true; // (not used in c++)
        k = 0;                          // Start with full speed (index [0])
        n = i*F_FSTR+1;
        i = i+1;

        // AMCL data (around 3Hz) - update to AMCL data if new AMCL pose received
        // otherwise make a guess
        if (this_amcl_x == prev_amcl_x && this_amcl_y == prev_amcl_y && this_amcl_theta == prev_amcl_theta) {
            // No AMCL update, so estimate initial values for prediction
            ropod_x = pred_x_ropod[1];
            ropod_y = pred_y_ropod[1];
            ropod_theta = pred_plan_theta[1];
        } else {
            // Set latest values from AMCL to previous AMCL values for next iteration
            // And take AMCL values as initial for the prediction
            prev_amcl_x = this_amcl_x;
            prev_amcl_y = this_amcl_y;
            prev_amcl_theta = this_amcl_theta;
            ropod_x = this_amcl_x;
            ropod_y = this_amcl_y;
            ropod_theta = this_amcl_theta;
        }

        //ROS_INFO("Ropod x: %f / Ropod y: %f / Theta: %f", ropod_x, ropod_y, ropod_theta);
        //ROS_INFO("xdot: %f / ydot: %f / thetadot %f", odom_xdot_ropod_global, odom_ydot_ropod_global, odom_thetadot_global);
        //ROS_INFO("ropodx: %f / ropody: %f / ropodtheta %f", ropod_x, ropod_y, ropod_theta);


        while ((ropod_colliding_obs || ropod_colliding_wall) && k < V_SCALE_OPTIONS.size())
        {
            v_scale = V_SCALE_OPTIONS[k];
            k++;
            m = 0;
            j = 0;
            t_pred[m] = 0;
            t_pred_j[j] = 0;

            //ROS_INFO("xdot: %f \t ydot: %f", odom_xdot_ropod_global, odom_ydot_ropod_global);

            // Initialize prediction with latest sim values
            pred_phi[0] = odom_phi_local; // On ropod
            pred_theta[0] = ropod_theta;
            pred_v_ropod[0] = control_v;
            if (abs(odom_vropod_global-control_v) > 0.5) {
                ROS_INFO("Difference between control and actual velocity > 0.5, correcting now.");
                pred_v_ropod[0] = odom_vropod_global;
            }
            pred_xdot[0] = pred_v_ropod[0]*cos(pred_phi[0])*cos(ropod_theta);   // xdot of rearaxle in global frame
            pred_ydot[0] = pred_v_ropod[0]*cos(pred_phi[0])*sin(ropod_theta);   // ydot of rearaxle in global frame
            pred_thetadot[0] = pred_v_ropod[0]*1/D_AX*sin(pred_phi[0]);
            pred_x_rearax[0] = ropod_x-D_AX*cos(ropod_theta);
            pred_y_rearax[0] = ropod_y-D_AX*sin(ropod_theta);
            pred_xy_ropod[0].x = ropod_x;
            pred_xy_ropod[0].y = ropod_y;
            prev_pred_phi_des = prev_sim_phi_des;
            pred_phi_des[0] = prev_pred_phi_des;
            pred_tube_width[0] = prev_sim_tube_width;
            pred_plan_theta[0] = ropod_theta;
            pred_v_ropod_plan[0] = pred_v_ropod[0];
            pred_state[0] = prev_sim_state;
            pred_task_counter[0] = prev_sim_task_counter;
            pred_x_obs[0] = current_obstacle.pose.position.x;
            pred_y_obs[0] = current_obstacle.pose.position.y;

            // Initialize areas
            u = pred_task_counter[0];
            if (u < ka_max-1)
            {
                area1ID = assignment[u];
                task1 = OBJ_X_TASK[u];
                area2ID = assignment[u+1];
                task2 = OBJ_X_TASK[u+1];
                if(u == (ka_max-2) )
                {
                    area3ID = assignment[u+1];
                    task3 = OBJ_X_TASK[u+1];
                }
                else
                {
                    area3ID = assignment[u+2];
                    task3 = OBJ_X_TASK[u+2];
                }
            }


            // Printing position
            // ROS_INFO("X: %f, Y: %f, Theta: %f", ropod_x, ropod_y, ropod_theta);

            // Prediction
            while (t_pred[m] < T_MIN_PRED) {
                j = j+1;

                // j and m counter are initialized at 0 instead of 1 in Matlab so we dont have to change their indices
                consider_overtaking_current_hallway = false;
                consider_overtaking_next_hallway = false;
                pred_tube_width[j] = pred_tube_width[j-1];  // Assume same as previous, might change later


                updateAreasAndFeatures();


                /**
                 * Check whether robot is entrying a new area
                 * */
                // Original implementation was checking for overlap between ropod shape and entry shape,
                // maybe change later if this implementation causes strange behavior
                //if(j==1)printf("Areas type curr next: %s, %s\n",curr_area.type.c_str(),next_area.type.c_str());
                pred_ropod_on_entry_inter[j] = false;
                pred_ropod_on_entry_hall[j] = false;
                if (curr_area.type == "hallway" && next_area.type == "inter")
                {                   
                    point_front = getPointByID(task1[1],pointlist);
                    local_wallpoint_front = coordGlobalToRopod(point_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    if(local_wallpoint_front.x < ENTRY_LENGTH) 
                        pred_ropod_on_entry_inter[j] = true;
                    else
                        pred_ropod_on_entry_inter[j] = false;
                        
                }
                else if (area1ID!=area2ID && curr_area.type == "hallway" && next_area.type == "hallway")
                {
                    point_front = getPointByID(task1[1],pointlist);
                    local_wallpoint_front = coordGlobalToRopod(point_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    if(local_wallpoint_front.x < SIZE_FRONT_RAX) 
                        pred_ropod_on_entry_hall[j] = true;
                    else
                        pred_ropod_on_entry_hall[j] = false;
                        
                }

                update_state_points = true; // Initially true, might change to false when not necessary
                prevstate = j-1;


                /**
                 * Based on predicted position, state, areas, features, update state and task
                 * */
                updateStateAndTask();
                /**
                 * Check wether an overtake should be considered at all
                 * */
                considerOvertaking();

                /**
                 * Call overtake state machine when needed
                 * */
                if ((pred_state[prevstate] == CRUSING && consider_overtaking_current_hallway) || consider_overtaking_next_hallway && (pred_state[prevstate] == TURNING || pred_state[prevstate] == GOING_STRAIGHT_ON_INTERSECTION))
                {
                    overtakeStateMachine();
                }
                else if (!consider_overtaking_current_hallway && !consider_overtaking_next_hallway)
                {
                    pred_tube_width[j] = TUBE_WIDTH_C;
                }


                /**
                 * Compute steering and default forward acceleration based on computed state and local features
                 * */
                computeSteeringAndVelocity();

                /**
                 * Simulate robot during current prediction step.
                 * Simulations are stacked over the different prediction steps.
                 * */
                simulateRobotDuringCurrentPredictionStep();

                /**
                 * Check for collision
                 * Either for obstacles or virtual walls(to be added)
                 * */
                checkForCollisions();
                if(ropod_colliding_obs || ropod_colliding_wall) // Do not execute current plan when collision is precited
                    break;


                visualizeRopodMarkers();

                // Update positions used to make the prediction plan with
                // Cesar-> TODO: In theory D_AX should be replaced by ROPOD_TO_AX, but it brings issues.
                //               Or rename predictions not to ropod but to steering point?
                pred_x_ropod[j] = pred_x_rearax[m]+D_AX*cos(pred_theta[m]);
                pred_y_ropod[j] = pred_y_rearax[m]+D_AX*sin(pred_theta[m]);
                pred_xy_ropod[j].x = pred_x_ropod[j];
                pred_xy_ropod[j].y = pred_y_ropod[j];
                pred_v_ropod_plan[j] = pred_v_ropod[m];

                if (j == 1) {
                    //ROS_INFO("v[0] = %f, a[1] = %f, ts = %f, v[1] = %f", pred_v_ropod[0], pred_accel[j], TS, pred_v_ropod_plan[j]);
                    // ROS_INFO("v[0] = %f, a[1] = %f, deltav = %f, v[1] = %f", pred_v_ropod[0], pred_accel[j], pred_accel[j]*F_PLANNER, pred_v_ropod[0]+pred_accel[1]*F_PLANNER);
                }
                pred_plan_theta[j] = pred_theta[m];
                //ROS_INFO("j: %d / State: %d / Time: %f / Phi: %f / V_des: %f", j, pred_state[j], t_pred_j[j], pred_phi_des[j], pred_v_ropod[j]);

            } // endwhile prediction

        }          // end while finding v_scale

        // Update after a prediction is made where no collision is caused
        // The prediction is ran until t_min_pred, however, the ropod will run a
        // new prediction the next step, so only the first part of the
        // prediction is used.

        prev_sim_state = pred_state[1];
        prev_sim_task_counter = pred_task_counter[1];
        prev_sim_phi_des = pred_phi_des[1];
        prev_sim_tube_width = pred_tube_width[1];
        program_duration = ( std::clock() - start_loop ) / (double) CLOCKS_PER_SEC;
        real_time_est = real_time_est+1/F_PLANNER;

        control_v = pred_v_ropod[0]+pred_accel[1]*1/F_PLANNER;
        // Compute v_ax and theta_dot from v_des and phi

        myfile << real_time_est << "\t" << program_duration << "\t" << pred_state[0] << "\t" << pred_task_counter[0] << "\t" << pred_tube_width[0] << "\t" <<
            pred_phi_des[0] << "\t" << pred_v_ropod[0] << "\t" << control_v << "\t"  <<  pred_xy_ropod[0].x << "\t" <<  pred_xy_ropod[0].y << "\t" <<
            pred_plan_theta[0] << "\t" << pred_x_obs[0] << "\t" << pred_y_obs[0] << "\t" << obs_theta << "\t" << current_obstacle.width << "\t" <<
            current_obstacle.depth << "\t" << current_obstacle.vel.x << "\t" << current_obstacle.vel.y << "\t" << pred_accel[1] << "\t" <<"\n";
        /*
        //ROS_INFO("Phi: %f / V_ax: %f / Theta_dot: %f", pred_phi_des[1], v_ax, theta_dot);
        ROS_INFO("state: %d, tube width: %f", pred_state[1], pred_tube_width[1]);
        ROS_INFO("K: %d", k);
        ROS_INFO("V desired: %f", pred_v_ropod_plan[1]);
        ROS_INFO("Predphi[1]: %f / [2]: %f / [3]: %f / [4]: %f", pred_phi_des[1], pred_phi_des[2], pred_phi_des[3], pred_phi_des[4]);
        */

        // if (pred_v_ropod_plan[1] > 0) {
        //     v_ax = cos(pred_phi_des[1])*pred_v_ropod_plan[1];
        //     theta_dot = pred_v_ropod_plan[1]/D_AX*sin(pred_phi_des[1]);
        //     publishCustomVelocity(v_ax, theta_dot);
        // } else {
        //     publishZeroVelocity();
        // }
        if (control_v > 0) {
            v_ax = cos(pred_phi_des[1])*control_v;
            theta_dot = control_v/D_AX*sin(pred_phi_des[1]);
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = v_ax;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = theta_dot;
            vel_pub.publish(cmd_vel);
        } else {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            vel_pub.publish(cmd_vel);
        }

        if (prev_sim_task_counter == (ka_max-1) ) {

            point_front = getPointByID(task1[1],pointlist);
            local_wallpoint_front = coordGlobalToRopod(point_front, pred_xy_ropod[1], pred_plan_theta[1]);
            if(local_wallpoint_front.x < REACHEDTARGETTRESHOLD)
            {
                ropod_reached_target = true;
                ROS_INFO("Ropod has reached its target, yay!");
            }
        }

        // Publish ropod points to rostopic
        vis_points.id = 2;
        vis_points.color.r = 0.0;
        vis_points.color.g = 0.0;
        vis_points.color.b = 1.0;
        vis_points.scale.x = 0.2;
        vis_points.scale.y = 0.2;
        geometry_msgs::Point vis_p;
        vis_p.x = ropod_x;
        vis_p.y = ropod_y;
        vis_points.points.push_back(vis_p);
        vis_points.id = 3;
        vis_points.color.r = 1.0;
        vis_points.color.g = 0.0;
        vis_points.color.b = 0.0;
        x_rearax = ropod_x - D_AX*cos(ropod_theta); // X position of center of rear axle [m]
        y_rearax = ropod_y - D_AX*sin(ropod_theta); // Y position of center of rear axle [m]
        vis_rt.x = x_rearax+(D_AX+SIZE_FRONT_ROPOD)*cos(ropod_theta)+SIZE_SIDE*cos(ropod_theta-0.5*M_PI);
        vis_rt.y = y_rearax+(D_AX+SIZE_FRONT_ROPOD)*sin(ropod_theta)+SIZE_SIDE*sin(ropod_theta-0.5*M_PI);
        vis_lt.x = x_rearax+(D_AX+SIZE_FRONT_ROPOD)*cos(ropod_theta)+SIZE_SIDE*cos(ropod_theta+0.5*M_PI);
        vis_lt.y = y_rearax+(D_AX+SIZE_FRONT_ROPOD)*sin(ropod_theta)+SIZE_SIDE*sin(ropod_theta+0.5*M_PI);
        vis_fr.x = FEELER_SIZE_STEERING*cos(ropod_theta+prev_sim_phi_des);
        vis_fr.y = FEELER_SIZE_STEERING*sin(ropod_theta+prev_sim_phi_des);
        vis_fl.x = FEELER_SIZE_STEERING*cos(ropod_theta+prev_sim_phi_des);
        vis_fl.y = FEELER_SIZE_STEERING*sin(ropod_theta+prev_sim_phi_des);
        vis_fr = vis_fr.add(vis_rt); vis_fl = vis_fl.add(vis_lt);
        vis_p.x = vis_lt.x; vis_p.y = vis_lt.y; vis_points.points.push_back(vis_p);
        vis_p.x = vis_rt.x; vis_p.y = vis_rt.y; vis_points.points.push_back(vis_p);
        vis_p.x = vis_fr.x; vis_p.y = vis_fr.y; vis_points.points.push_back(vis_p);
        vis_p.x = vis_fl.x; vis_p.y = vis_fl.y; vis_points.points.push_back(vis_p);
        vis_points.id = 4;
        vis_points.color.r = 0.0;
        vis_points.color.g = 1.0;
        vis_points.color.b = 0.0;
        vis_p.x = glob_wallpoint_rear.x; vis_p.y = glob_wallpoint_rear.y; vis_points.points.push_back(vis_p);
        vis_p.x = glob_wallpoint_front.x; vis_p.y = glob_wallpoint_front.y; vis_points.points.push_back(vis_p);
        ropodmarker_pub.publish(vis_points);
        vis_points.points.clear();
        // End publish ropod points

        }   // end if received goal

        ros::spinOnce();
        rate.sleep();
        if(rate.cycleTime() > ros::Duration(1/F_PLANNER) ){
            ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", F_PLANNER, rate.cycleTime().toSec());
        }

    }
    // Will only perform this when ropod has reached target
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub.publish(cmd_vel);
    myfile.close();

    return 0;
}
