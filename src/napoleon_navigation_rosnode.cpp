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
#include <dynamic_reconfigure/server.h>
#include <napoleon_navigation/NapoleonNavigationConfig.h>
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
#include <std_msgs/Float64.h>
#include <stdlib.h>

std::vector<PointID> pointlist;
std::vector<AreaQuadID> arealist;
std::vector<int> assignment;

double ropod_x = 0, ropod_y = 0, ropod_theta = 0;
double this_amcl_x = 0, this_amcl_y = 0, quaternion_x = 0, quaternion_y = 0, quaternion_z = 0, quaternion_w = 0, this_amcl_theta = 0, siny_cosp = 0, cosy_cosp = 0;
double odom_xdot_ropod_global = 0, odom_ydot_ropod_global = 0, odom_thetadot_global = 0, odom_phi_local = 0, odom_phi_global = 0, odom_vropod_global = 0;
double FutureTimeStamp = 0.75;
int no_obs = 0;
ed_gui_server::objPosVel current_obstacle;
double obs_theta = 0.0;
Point obs_center_global;
Rectangle freeNavigationRightLaneRight;
Rectangle freeNavigationLeftLaneLeft;
Rectangle freeNavigationCenter;
double steering_offset;
double program_duration = 0, real_time_est = 0;

//std::vector<geometry_msgs::PoseStamped> global_path;
bool start_navigation = false;
geometry_msgs::PoseStamped simple_goal;

bool action_server_enabled =  false;
bool goal_received = false;
ropod_ros_msgs::RoutePlannerResult debug_route_planner_result_;

struct NapoleonConfig config;

void getObstaclesCallback(const ed_gui_server::objsPosVel::ConstPtr& obsarray)
{
    //#define RECTANGULAR_MARGIN 0.25
    //#define RECTANGULAR_DESIRED_DIM 1.0

    //ROS_INFO("%lu obstacles detected", obsarray->objects.size());

    //// For now, only take all obstacles which are assumed to be a rectangle with
    //// one of the dimensions having a size of approximately 1m.
    //// Then, take the closest one.

    //float closestDist = std::numeric_limits< float >::infinity();
    //unsigned int IclosestObject;
    //bool rectangleDetected = false;

    //for(unsigned int iObs = 0; iObs < obsarray->objects.size(); iObs++)
    //{
        //ed_gui_server::objPosVel candidate_obstacle = obsarray->objects[iObs];

        //bool check1 = candidate_obstacle.rectangle.probability > 0.5 ;
        //bool check2 = std::fabs(candidate_obstacle.rectangle.width - RECTANGULAR_DESIRED_DIM) < RECTANGULAR_MARGIN ;
        //bool check3 = std::fabs(candidate_obstacle.rectangle.depth - RECTANGULAR_DESIRED_DIM) < RECTANGULAR_MARGIN ;

        //std::cout << "Checks = " << check1 << check2 << check3 << std::endl;
        //std::cout << "candidate_obstacle.rectangle.probability = " << candidate_obstacle.rectangle.probability << std::endl;
        //std::cout << "candidate_obstacle.rectangle.width = " << candidate_obstacle.rectangle.width << std::endl;
        //std::cout << "candidate_obstacle.rectangle.depth = " << candidate_obstacle.rectangle.depth << std::endl;

        //if( candidate_obstacle.rectangle.probability > 0.5 &&
            //( std::fabs(candidate_obstacle.rectangle.width - RECTANGULAR_DESIRED_DIM) < RECTANGULAR_MARGIN ||
              //std::fabs(candidate_obstacle.rectangle.depth - RECTANGULAR_DESIRED_DIM) < RECTANGULAR_MARGIN ) )
        //{
                //float dist = std::pow(candidate_obstacle.rectangle.pose.position.x - this_amcl_x, 2.0) + std::pow(candidate_obstacle.rectangle.pose.position.y - this_amcl_y, 2.0);

                //if(dist < closestDist)
                //{

                        //closestDist = dist;
                        //IclosestObject = iObs;
                        //rectangleDetected = true;
                //}
        //}
    //}

    //if(rectangleDetected)
    //{
        //current_obstacle = obsarray->objects[IclosestObject];
        //ROS_INFO("Clostest object: Obs is %f wide and %f deep", current_obstacle.rectangle.width, current_obstacle.rectangle.depth);
        //ROS_INFO("Obs x: %f, obs y: %f", current_obstacle.rectangle.pose.position.x, current_obstacle.rectangle.pose.position.y);
        //ROS_INFO("Vx: %f, Vy %f", current_obstacle.rectangle.vel.x, current_obstacle.rectangle.vel.y);
        //quaternion_x = obsarray->objects[IclosestObject].rectangle.pose.orientation.x;
        //quaternion_y = obsarray->objects[IclosestObject].rectangle.pose.orientation.y;
        //quaternion_z = obsarray->objects[IclosestObject].rectangle.pose.orientation.z;
        //quaternion_w = obsarray->objects[IclosestObject].rectangle.pose.orientation.w;

        //// yaw (z-axis rotation)
        //siny_cosp = +2.0 * (quaternion_w * quaternion_z + quaternion_x * quaternion_y);
        //cosy_cosp = +1.0 - 2.0 * (quaternion_y * quaternion_y + quaternion_z * quaternion_z);
        //obs_theta = atan2(siny_cosp, cosy_cosp);
        //obs_center_global.x = current_obstacle.rectangle.pose.position.x;
        //obs_center_global.y = current_obstacle.rectangle.pose.position.y;
    //}
}

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
    //ROS_INFO("X: %f, Y: %f", pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
    this_amcl_x = pose_msg->pose.pose.position.x;
    this_amcl_y = pose_msg->pose.pose.position.y;
    this_amcl_theta = tf::getYaw(pose_msg->pose.pose.orientation);
}

void getFutureTimeStampCallback(const std_msgs::Float64::ConstPtr& future_time_stamp)
{
    FutureTimeStamp = future_time_stamp->data;
}

void simpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
    ROS_INFO("new simple goal received");
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
    vis_wall.color.r = 0.1;
    vis_wall.color.g = 0.9;
    vis_wall.color.b = 0.1;
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

void showPivotPoint(Point local_wallpoint_front, ros::Publisher &pub) {
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
    vis_wall.color.r = 0.9;
    vis_wall.color.g = 0.1;
    vis_wall.color.b = 0.1;
    vis_wall.type = visualization_msgs::Marker::POINTS;
    vis_wall.scale.x = 0.3;
    vis_wall.scale.y = 0.3;
    geometry_msgs::Point wall_p;

    vis_wall.points.clear();
    wall_p.x =  local_wallpoint_front.x;
    wall_p.y =  local_wallpoint_front.y;
    vis_wall.points.push_back(wall_p);
    //wall_p.x =  local_wallpoint_rear.x;
    //wall_p.y =  local_wallpoint_rear.y;
    //vis_wall.points.push_back(wall_p);
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
double pred_sim_x_ropod[size_m] {x_ropod_0};
double pred_sim_y_ropod[size_m] {y_ropod_0};
double pred_x_obs[size_p] {0};
double pred_y_obs[size_p] {0};
double v_obs_sq = 0;
double prev_sim_phi_des;
double pred_plan_theta[size_p] {theta_0};
double pred_accel[size_p] {0};
std::vector<double> sim_theta {pred_plan_theta[0]};
std::vector<double> sim_phi {phi_0};
std::vector<double> sim_v_scale {1};
double prev_sim_tube_width {config.TUBE_WIDTH_C};
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
double pred_tube_width[size_p] {config.TUBE_WIDTH_C};
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
double max_delta_v = config.A_MAX*delta_t;             // Maximum change in v in delta_t
double lpf = fmax(1.0,2*M_PI*TS*config.CUTOFF_FREQ);             // Low pass filter [-]
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
bool overtake_on_current_hallway;
bool update_assignment;
int uprev;
bool ropod_colliding_obs = true;
bool ropod_colliding_wall = true;
bool robot_left_side_wall_first_time = false;


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

Point local_front_ropod_dilated_p0(config.DILATE_ROPOD_ALIGNING, -config.DILATE_ROPOD_ALIGNING);
Point local_front_ropod_dilated_p1(config.DILATE_ROPOD_ALIGNING, config.DILATE_ROPOD_ALIGNING);

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
double obj2wall_angle, obj3wall_angle, relative_angle;

std::vector<AreaQuad> robot_footprint_list;


void dynamicReconfigureCallback(napoleon_navigation::NapoleonNavigationConfig &dyn_config, uint32_t level)
    {
        config.SIZE_SIDE = dyn_config.size_side;
        config.ROPOD_LENGTH = dyn_config.ropod_length;
        config.SIZE_FRONT_ROPOD = config.ROPOD_LENGTH / 2.0;
        config.FEELER_SIZE = dyn_config.feeler_size;
        config.FEELER_SIZE_STEERING = dyn_config.feeler_size_steering;
        config.ENV_TCTW_SIZE = dyn_config.env_tctw_size;
        config.ENV_TRNS_SIZE = dyn_config.env_trns_size;
        config.CARROT_LENGTH = config.FEELER_SIZE + dyn_config.carrot_length_feeler_offset;
        config.ENV_TRNS_SIZE_CORNERING = dyn_config.env_trns_size_cornering;
        config.V_CRUISING = dyn_config.v_cruising;
        config.V_INTER_TURNING = dyn_config.v_inter_turning;
        config.V_INTER_ACC = config.V_INTER_TURNING;
        config.V_INTER_DEC = config.V_INTER_TURNING;
        config.V_ENTRY = config.V_INTER_TURNING;
        config.V_STEERSATURATION = dyn_config.v_steersaturation;
        config.DELTA_DOT_LIMIT = dyn_config.delta_dot_limit;
        config.A_MAX = dyn_config.a_max;
	// config.A_START = dyn_config.a_start;
        config.V_OBS_OVERTAKE_MAX = dyn_config.v_obs_overtake_max;
        config.MIN_DIST_TO_OVERTAKE = dyn_config.min_dist_to_overtake;
        config.START_STEERING_EARLY_RIGHT = dyn_config.start_steering_early_right;
        config.START_STEERING_EARLY_LEFT = dyn_config.start_steering_early_left;
        config.ROTATED_ENOUGH_THRES = dyn_config.rotated_enough_thres;
        if (dyn_config.is_load_attached)
        {
            config.D_AX = dyn_config.d_ax;
            config.ROPOD_TO_AX = config.D_AX;
            config.SIZE_REAR = dyn_config.size_rear;
            config.V_OVERTAKE = dyn_config.v_overtake;
        }
        else
        {
            config.D_AX = config.ROPOD_LENGTH;
            config.ROPOD_TO_AX = 0.0;
            config.SIZE_REAR = config.ROPOD_LENGTH / 2.0;
            config.V_OVERTAKE = 0.8 * config.V_CRUISING;
        }
        config.SIZE_FRONT_RAX = (config.ROPOD_TO_AX + config.SIZE_FRONT_ROPOD);
        config.FOLLOW_WALL_DIST_TURNING = sqrt((config.ROPOD_LENGTH * config.ROPOD_LENGTH) / 2.0) + config.ENV_TCTW_SIZE + config.ENV_TRNS_SIZE;
        config.T_MIN_PRED = dyn_config.t_min_pred;
        config.T_PRED_WALL_COLLISION = dyn_config.t_pred_wall_collision;
        config.T_PRED_OBS_COLLISION = dyn_config.t_pred_obs_collision;

        config.CUTOFF_FREQ = dyn_config.cutoff_freq;
        lpf = fmax(1.0,2*M_PI*TS*config.CUTOFF_FREQ);             // Low pass filter [-]

        config.OBS_AVOID_MARGIN = dyn_config.obs_avoid_margin;
        config.OBS_AVOID_MARGIN_FRONT = dyn_config.obs_avoid_margin_front;

        config.DILATE_ROPOD_ALIGNING = dyn_config.dilate_ropod_aligning;
        local_front_ropod_dilated_p0.x = config.DILATE_ROPOD_ALIGNING;
        local_front_ropod_dilated_p0.y = -config.DILATE_ROPOD_ALIGNING;
        local_front_ropod_dilated_p1.x = config.DILATE_ROPOD_ALIGNING;
        local_front_ropod_dilated_p1.y = config.DILATE_ROPOD_ALIGNING;

        config.TUBE_WIDTH_C = 2.0 * (config.SIZE_SIDE + config.ENV_TCTW_SIZE + fmax(config.ENV_TRNS_SIZE, config.OBS_AVOID_MARGIN));
        config.REACHEDTARGETTHRESHOLD = dyn_config.reached_target_threshold;
        config.ENTRY_LENGTH = dyn_config.entry_length;
        config.SHARP_ANGLE_THRESHOLD = dyn_config.sharp_angle_threshold;

    }

/******************************************
 * Initialization for assigment
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
    if (ka_max < 3) 
    {
      ROS_ERROR("Need at least 3 areas for assignment");
    }
    
    AreaQuadID OBJ1 = getAreaByID(assignment[0],arealist);
    AreaQuadID OBJ2 = getAreaByID(assignment[1],arealist);
    AreaQuadID OBJ3 = getAreaByID(assignment[2],arealist);
    int obj2tasklen;

    for (int ka = 0; ka < ka_max; ka++) {
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
            if (obj2tasklen > 5) {
                area_names.push_back(OBJ2TASK[2]);
                area_names.push_back(OBJ2TASK[3]);
                area_names.push_back(OBJ2TASK[4]);
                area_names.push_back(OBJ2TASK[5]);
                area_names.push_back(OBJ2TASK[6]);
                area_names.push_back(OBJ2TASK[7]);

                obj2wall_p0 = getPointByID(OBJ2TASK[0],pointlist);
                obj2wall_p1 = getPointByID(OBJ2TASK[1],pointlist);
                obj3wall_p0 = getPointByID(OBJ2TASK[2],pointlist);
                obj3wall_p1 = getPointByID(OBJ2TASK[3],pointlist);

                obj2wall_angle = atan2(obj2wall_p1.y-obj2wall_p0.y, obj2wall_p1.x-obj2wall_p0.x);
                obj3wall_angle = atan2(obj3wall_p1.y-obj3wall_p0.y, obj3wall_p1.x-obj3wall_p0.x);
                relative_angle = wrapToPi(obj3wall_angle-obj2wall_angle);

                if (OBJ2TASK[5].compare("right") == 0 && relative_angle < -config.SHARP_ANGLE_THRESHOLD) {
                    // Sharp angle to the right, we need to take the next wall into account as well
                    sharp_corner[ka+1] = true;
                } else if (OBJ2TASK[5].compare("left") == 0 && relative_angle > config.SHARP_ANGLE_THRESHOLD) {
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



/**
 * Update areas and features (corner pivot locations, walls, etc)
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
                cur_next_hallway_rear = getPointByID(task2[6],pointlist);
                cur_next_hallway_front = getPointByID(task2[7],pointlist);
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
        if (real_time_est <= 1) {       // First second is reserved for departure, in which a backwards movement is made.
            pred_state[j] = DEPARTURE;  // Smarter statement is needed, so departure can also happen after ropod has stopped for some time and starts moving again
        //printf("Ropod entry hall task2[5] non empty: %d\n",(int)pred_ropod_on_entry_hall[j]);
        } else if (real_time_est >= 1 && pred_state[prevstate] == DEPARTURE){
            pred_state[j] = CRUISING;
        } else if (pred_ropod_on_entry_hall[j] && (pred_state[prevstate] == DEPARTURE || pred_state[prevstate] == CRUISING || pred_state[prevstate] == SPACIOUS_OVERTAKE || pred_state[prevstate] == TIGHT_OVERTAKE) ){                
            pred_state[j] = pred_state[prevstate];
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


        } else if (pred_ropod_on_entry_inter[j] && (pred_state[prevstate] == DEPARTURE || pred_state[prevstate] == CRUISING || pred_state[prevstate] == SPACIOUS_OVERTAKE || pred_state[prevstate] == TIGHT_OVERTAKE) ) {
                // If cruising and the y position of the ropod exceeds the y
                // position of the entry
                if(j==1)printf("Entry detected to turn intersection u = %d\n",u);
                pred_state[j] = ENTRY_BEFORE_TURN_ON_INTERSECTION;

                PointID point_right_entry_next_wall = getPointByID(task2[2],pointlist);
                point_pivot = getPointByID(task2[4],pointlist);
                double standard_steering_offset;
                // TODO: change calculation to perpendicular distances
                if(task2[5] == "right")
                {
                    if(j==1)printf("Will turn right");
                    standard_steering_offset =  fmin( 0.0, fabs(cur_pivot_local.y)
                                    - (sqrt(dist2(getPoint(point_right_entry_next_wall), getPoint(point_pivot))) - config.TUBE_WIDTH_C/2.0)
                                    ) + config.ROPOD_TO_AX;
                    steering_offset = 0.0;//standard_steering_offset; // + config.START_STEERING_EARLY_RIGHT
                }
                else
                {
                    if(j==1)printf("Will turn left");
                    standard_steering_offset = fmin( 0.0, fabs(cur_pivot_local.y)
                                    - (sqrt(dist2(getPoint(point_right_entry_next_wall), getPoint(point_pivot))) - config.TUBE_WIDTH_C/2.0)
                                    ) + config.ROPOD_TO_AX;
                    steering_offset = config.START_STEERING_EARLY_LEFT + standard_steering_offset;
                }

        } else if (cur_pivot_local.x < config.SIZE_FRONT_ROPOD + steering_offset && pred_state[prevstate] == ENTRY_BEFORE_TURN_ON_INTERSECTION) {
            // If in entry and the y position of the ropod exceeds the y
            // position of the intersection
            pred_state[j] = ACCELERATE_ON_INTERSECTION;
            


        } else if (cur_pivot_local.x <= config.SIZE_FRONT_RAX + steering_offset && pred_state[prevstate] == ACCELERATE_ON_INTERSECTION) {
            // If middle of vehicle is on y height of pivot
            pred_state[j] = ALIGN_AXIS_AT_INTERSECTION;
        } else if (cur_pivot_local.x <= -config.ROPOD_TO_AX + steering_offset && pred_state[prevstate] == ALIGN_AXIS_AT_INTERSECTION) {
            // If rearaxle is aligned with the pivot minus sse
            if(j==1)printf("Turning on  u = %d\n",u+1);
            pred_state[j] = TURNING;

        } else if (-config.ROTATED_ENOUGH_THRES < cur_next_hallway_angle && cur_next_hallway_angle < config.ROTATED_ENOUGH_THRES && pred_state[prevstate] == TURNING) {
            // If ropod has turned enough

            pred_state[j] = CRUISING;
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
    // Going straight on an intersection / or between hallways
    } else if (!task2[1].empty()) {      
	if (real_time_est <= 1) {       // First second is reserved for departure, in which a backwards movement is made.
            pred_state[j] = DEPARTURE;  // Smarter statement is needed, so departure can also happen after ropod has stopped for some time and starts moving again
        //printf("Ropod entry hall task2[5] non empty: %d\n",(int)pred_ropod_on_entry_hall[j]);
        } else if (real_time_est >= 1 && pred_state[prevstate] == DEPARTURE){
            pred_state[j] = CRUISING;
	} else if (pred_ropod_on_entry_hall[j] && pred_state[prevstate] == CRUISING) {
            pred_state[j] = CRUISING;
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


        }else if (pred_ropod_on_entry_inter[j] && pred_state[prevstate] == CRUISING) {
            // If cruising and the y position of the ropod exceeds the y
            // position of the entry
            if(j==1)printf("Entry detected to straight intersection u = %d\n",u);
            pred_state[j] = ENTRY_BEFORE_GOING_STRAIGHT_ON_INTERSECTION;
        } else if (current_inter_rear_wall_local.x < config.SIZE_FRONT_ROPOD + steering_offset && pred_state[prevstate] == ENTRY_BEFORE_GOING_STRAIGHT_ON_INTERSECTION) {
            // If in entry and the y position of the ropod exceeds the y
            // position of the intersection
            if(j==1)printf("Straight on u = %d\n",u+1);
            pred_state[j] = GOING_STRAIGHT_ON_INTERSECTION;
        } else if (current_inter_front_wall_local.x < -config.D_AX/2 && pred_state[prevstate] == GOING_STRAIGHT_ON_INTERSECTION) {

            pred_state[j] = CRUISING;
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
	if(j==1)printf("No obs counter: %d \n", no_obs);
        if (no_obs > 0) {
            current_obs_in_ropod_frame_pos = coordGlobalToRopod(obs_center_global, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
            //disp(['Obs is ',num2str(obs_in_ropod_frame_pos.x), ' m in front of ropod']);
            if (current_obs_in_ropod_frame_pos.x+current_obstacle.rectangle.depth/2+config.D_AX+config.SIZE_REAR < 0) {
		if(j==1) printf("obstacle avoided 1");
                pred_state[j] = CRUISING;
                update_state_points = true;
                pred_tube_width[j] = config.TUBE_WIDTH_C;
		no_obs = 0;
            } else {
                pred_state[j] = pred_state[prevstate];
                update_state_points = false;
            }
        } else {
            // Ropod doesn't see obs anymore, return cruising.
            // When obs disappears depends on the setting how long it takes before obs disappears after not seeing it (entity-timeout)
            // It can be found in /catkin_workspace/src/applications/ropod_navigation_test/config/model-example-ropod-navigation-ED.yaml
	    if(j==1) printf("obstacle avoided 2");
            pred_state[j] = CRUISING;
            update_state_points = true;
            pred_tube_width[j] = config.TUBE_WIDTH_C;
        }
    }

    // TODO: This code was moved. It was right before steerings were computed based on prediction. Check if still works on all cases
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
ros::Publisher pivotmarker_pub;
ros::Publisher freeAreaOR_marker_pub;
ros::Publisher freeAreaOL_marker_pub;
/**
 * Compute steering and default forward acceleration based on computed state and local features
 * */
void computeSteeringAndVelocity()
{
    // Perform the appropriate action according to finite state machine
    if (pred_state[j] == DEPARTURE || pred_state[j] == CRUISING || pred_state[j] == GOING_STRAIGHT_ON_INTERSECTION || pred_state[j] == TIGHT_OVERTAKE || pred_state[j] == SPACIOUS_OVERTAKE) {
        // disp([num2str[j],' - Ropod is now cruising']);
        if (pred_state[j] == DEPARTURE) { // Departure
            if (j==1) printf("DEPARTURE\n");
            if (update_state_points) {
                point_rear = getPointByID(task1[0],pointlist);
                point_front = getPointByID(task1[1],pointlist);
                wallang = atan2(point_front.y-point_rear.y,point_front.x-point_rear.x);
                glob_wallpoint_front.x = point_front.x+shift_wall*cos(wallang+M_PI/2);
                glob_wallpoint_front.y = point_front.y+shift_wall*sin(wallang+M_PI/2);
                glob_wallpoint_rear.x = point_rear.x+shift_wall*cos(wallang+M_PI/2);
                glob_wallpoint_rear.y = point_rear.y+shift_wall*sin(wallang+M_PI/2);
            }
            v_des = -config.V_CRUISING;                
        } else if (pred_state[j] == CRUISING) {   // Cruising up
            if(j==1) printf("CRUISING\n");
            if (update_state_points) {
                point_rear = getPointByID(task1[0],pointlist);
                point_front = getPointByID(task1[1],pointlist);
                wallang = atan2(point_front.y-point_rear.y,point_front.x-point_rear.x);
                glob_wallpoint_front.x = point_front.x+shift_wall*cos(wallang+M_PI/2);
                glob_wallpoint_front.y = point_front.y+shift_wall*sin(wallang+M_PI/2);
                glob_wallpoint_rear.x = point_rear.x+shift_wall*cos(wallang+M_PI/2);
                glob_wallpoint_rear.y = point_rear.y+shift_wall*sin(wallang+M_PI/2);
            }
            v_des = config.V_CRUISING;
        } else if (pred_state[j] == GOING_STRAIGHT_ON_INTERSECTION) { // Straight on inter
            if(j==1) printf("GOING_STRAIGHT_ON_INTERSECTION\n");
            if (update_state_points) {
                point_rear = getPointByID(task2[0],pointlist);
                point_front = getPointByID(task2[1],pointlist);
                glob_wallpoint_front.x = point_front.x;
                glob_wallpoint_front.y = point_front.y;
                glob_wallpoint_rear.x = point_rear.x;
                glob_wallpoint_rear.y = point_rear.y;
            }
            v_des = config.V_INTER_ACC;
        } else if (pred_state[j] == TIGHT_OVERTAKE) { // Tight overtake
            if(j==1) printf("TIGHT_OVERTAKE\n");
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
            v_des = config.V_OVERTAKE;
        } else if (pred_state[j] == SPACIOUS_OVERTAKE) { // Spacious overtake
            if(j==1) printf("SPACIOUS_OVERTAKE\n");
            
            point_rear = getPointByID(task1[0],pointlist);
            point_front = getPointByID(task1[1],pointlist);
            wallang = atan2(point_front.y-point_rear.y,point_front.x-point_rear.x);
            //glob_wallpoint_front = [point_front.x, point_front.y]+shift_wall*[cos(wallang+M_PI/2), sin(wallang+M_PI/2)];
            //glob_wallpoint_rear = [point_rear.x, point_rear.y]+shift_wall*[cos(wallang+M_PI/2), sin(wallang+M_PI/2)];
            glob_wallpoint_front.x = point_front.x+shift_wall*cos(wallang+M_PI/2);
            glob_wallpoint_front.y = point_front.y+shift_wall*sin(wallang+M_PI/2);
            glob_wallpoint_rear.x = point_rear.x+shift_wall*cos(wallang+M_PI/2);
            glob_wallpoint_rear.y = point_rear.y+shift_wall*sin(wallang+M_PI/2);
            v_des = config.V_OVERTAKE;
        }
        local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        if(j==1) showWallPoints(local_wallpoint_front, local_wallpoint_rear, wallmarker_pub);
        double distance_point_to_line = -distToLine(pred_xy_ropod[j-1], glob_wallpoint_rear, glob_wallpoint_front);
        double carrot_length = config.CARROT_LENGTH;
        if(distance_point_to_line < 0)
        {
            //if(j==1) printf("DECREASE CARROT\n");
            //carrot_length = 0.3*CARROT_LENGTH; // this increases sharpness of corrections
        }

        pred_phi_des[j] = getSteering(local_wallpoint_front, local_wallpoint_rear, pred_tube_width[j], carrot_length, config.FEELER_SIZE );
    } else if (pred_state[j] == ENTRY_BEFORE_TURN_ON_INTERSECTION || pred_state[j] == ENTRY_BEFORE_GOING_STRAIGHT_ON_INTERSECTION) {
        if(j==1) printf("ENTRY_BEFORE_INTERSECTION\n");
        // disp([num2str[j],' - Ropod is now in entry']);
        if (update_state_points) {
            point_rear = getPointByID(task1[0],pointlist);
            point_front = getPointByID(task1[1],pointlist);
	    point_pivot = getPointByID(task1[4],pointlist);
            glob_wallpoint_front.x = point_front.x;
            glob_wallpoint_front.y = point_front.y;
            glob_wallpoint_rear.x = point_rear.x;
            glob_wallpoint_rear.y = point_rear.y;
        }
        local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
	local_pivot = coordGlobalToRopod(point_pivot, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        if(j==1)
	{
	  showWallPoints(local_wallpoint_front, local_wallpoint_rear, wallmarker_pub);
	  showPivotPoint(cur_pivot_local, wallmarker_pub);
	}
        pred_phi_des[j] = getSteering(local_wallpoint_front, local_wallpoint_rear, pred_tube_width[j], config.CARROT_LENGTH, config.FEELER_SIZE);
        v_des = config.V_INTER_TURNING + ((config.V_CRUISING - config.V_INTER_TURNING)/0.81)*local_pivot.x;//config.V_ENTRY;
	if(j==1) printf("Desired speed: %f \n", v_des);
    } else if (pred_state[j] == ACCELERATE_ON_INTERSECTION) {
        if(j==1) printf("ACCELERATE_ON_INTERSECTION\n");
        // disp([num2str[j],' - Ropod is at inter, driving forward']);
        if (update_state_points) {
            point_rear = getPointByID(task1[0],pointlist);
            point_front = getPointByID(task1[1],pointlist);
	    point_pivot = getPointByID(task1[4],pointlist);
            glob_wallpoint_front.x = point_front.x;
            glob_wallpoint_front.y = point_front.y;
            glob_wallpoint_rear.x = point_rear.x;
            glob_wallpoint_rear.y = point_rear.y;
        }

        local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
	local_pivot = coordGlobalToRopod(point_pivot, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        if(j==1) 
	{
	  showWallPoints(local_wallpoint_front, local_wallpoint_rear, wallmarker_pub);
	  showPivotPoint(cur_pivot_local, wallmarker_pub);
	}
        pred_phi_des[j] = getSteering(local_wallpoint_front, local_wallpoint_rear, pred_tube_width[j], config.CARROT_LENGTH, config.FEELER_SIZE);
        v_des = config.V_INTER_TURNING + ((config.V_CRUISING - config.V_INTER_TURNING)/0.81)*local_pivot.x;//config.V_INTER_ACC;
	if(j==1) printf("Desired speed: %f \n", v_des);
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
        if(j==1) printf("ALIGN_AXIS_AT_INTERSECTION\n");
        // disp([num2str[j],' - Ropod is at inter, driving forward']);
        if (update_state_points) {
            point_rear = getPointByID(task1[0],pointlist);
            point_front = getPointByID(task1[1],pointlist);
	    point_pivot = getPointByID(task1[4],pointlist);
            glob_wallpoint_front.x = point_front.x;
            glob_wallpoint_front.y = point_front.y;
            glob_wallpoint_rear.x = point_rear.x;
            glob_wallpoint_rear.y = point_rear.y;
        }

        local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
	local_pivot = coordGlobalToRopod(point_pivot, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
        if(j==1) 
	{
	  showWallPoints(local_wallpoint_front, local_wallpoint_rear, wallmarker_pub);
	  showPivotPoint(cur_pivot_local, wallmarker_pub);
	}
        pred_phi_des[j] = getSteering(local_wallpoint_front, local_wallpoint_rear, pred_tube_width[j], config.CARROT_LENGTH, config.FEELER_SIZE);
        v_des = config.V_INTER_TURNING + ((config.V_CRUISING - config.V_INTER_TURNING)/0.81)*local_pivot.x;//config.V_INTER_DEC;
	if(j==1) printf("Desired speed: %f \n", v_des);
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
        if(j==1) printf("TURNING\n");
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
            pred_phi_des[j] = getSteeringTurn(local_pivot, dir_cw, local_wallpoint_front, local_wallpoint_rear, config.CARROT_LENGTH, config.FEELER_SIZE);
        } else {
            //pred_phi_des[j] = getSteeringTurnSharp(pred_x_ropod(j-1), pred_y_ropod(j-1), pred_plan_theta[j-1], size_front_ropod, size_side, feeler_size_steering, d_ax, dir_cw, task2, pointlist, follow_wall_distance, env_tctw_size, env_trns_size_cornering, env_carrot_size);
            pred_phi_des[j] = getSteeringTurnSharp(pred_xy_ropod[j-1], pred_plan_theta[j-1], dir_cw, task2, pointlist, config.CARROT_LENGTH, config.FEELER_SIZE_STEERING);
        }
        if(j==1) showPivotPoint(local_pivot, wallmarker_pub);
        v_des = config.V_INTER_TURNING;
	if(j==1) printf("Desired speed: %f \n", v_des);
    }

    // Wrap to [-pi,pi] domain
    pred_phi_des[j] = wrapToPi(pred_phi_des[j]);

    // Saturate steering rate
    if (abs(pred_phi_des[j]-prev_pred_phi_des) > config.DELTA_DOT_LIMIT/(double)F_PLANNER) {
        //disp("Delta steering too large, steering saturated");
        pred_phi_des[j] = prev_pred_phi_des + sgn(pred_phi_des[j]-prev_pred_phi_des)*config.DELTA_DOT_LIMIT/(double)F_PLANNER;
        // Decrease vel leads to better corners
        // The velocity is already decreased in the state machine, but this is just a harsh backup
        // pred_steer_rate_saturation[j] = 1;
        if (v_des > config.V_STEERSATURATION) {
            v_des = config.V_STEERSATURATION;
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
        pred_accel[j] = sgn(v_des_scaled[j]-pred_v_ropod_plan[j-1])*config.A_MAX;
    }
    //ROS_INFO("pred_accel: %f", pred_accel[j]);
}

std_msgs::Float64 fut_phi;
ros::Publisher fut_phi_pub;
nav_msgs::Odometry pred_pose_vel;
ros::Publisher pred_pose_vel_pub;

/**
 * Simulate robot during current prediction step.
 * Simulations are stacked over the different prediction steps.
 * */
void simulateRobotDuringCurrentPredictionStep()
{
    m_prev = m;
    t_pred_prev = t_pred[m];
    t_pred_j[j] = t_pred_prev+F_FSTR*TS;
    double FutureTimeStampIndex = FutureTimeStamp/TS;
  
    // Simulate ropod motion with current plan (Simulation is done faster than controller sample time)
    for (int q = 1; q <= F_FSTR; ++q) { // q = [1, 2, ..., F_FSTR]
        m = m_prev+q;           // Current iteration
        t_pred[m] = t_pred_prev+q*TS;

        pred_phi[m] = (1-lpf)*pred_phi[m-1]+lpf*pred_phi_des[j];
        pred_theta[m] = wrapToPi(pred_theta[m-1]+pred_thetadot[m-1]*TS);
        pred_v_ropod[m] = pred_v_ropod[m-1]+pred_accel[j]*TS;

        pred_xdot[m] = pred_v_ropod[m]*cos(pred_phi[m])*cos(pred_theta[m]);
        pred_ydot[m] = pred_v_ropod[m]*cos(pred_phi[m])*sin(pred_theta[m]);
        pred_thetadot[m] = pred_v_ropod[m]*1/config.D_AX*sin(pred_phi[m]);
      
        pred_x_rearax[m] = pred_x_rearax[m-1]+pred_xdot[m]*TS;
        pred_y_rearax[m] = pred_y_rearax[m-1]+pred_ydot[m]*TS;
	pred_sim_x_ropod[m] = pred_x_rearax[m]+config.D_AX*cos(pred_theta[m]);
        pred_sim_y_ropod[m] = pred_y_rearax[m]+config.D_AX*sin(pred_theta[m]);
	
	pred_pose_vel.pose.pose.position.x = pred_sim_x_ropod[m];
	pred_pose_vel.pose.pose.position.y = pred_sim_y_ropod[m];
	pred_pose_vel.twist.twist.linear.x = pred_v_ropod[m]*cos(pred_phi[m]);
	pred_pose_vel.twist.twist.linear.y = t_pred[m];
	pred_pose_vel.twist.twist.angular.z = pred_thetadot[m];
	pred_pose_vel.header.stamp = ros::Time::now();;
	pred_pose_vel_pub.publish(pred_pose_vel);
	
	//Check if prediction index is equal to the desired future heading time
	//And if so, publish it.
	if(abs(m - FutureTimeStampIndex) < 0.5) //Inequality because we're comparing a double to an integer
	{
	  fut_phi.data = pred_phi[m];
	  fut_phi_pub.publish(fut_phi);
	  //ROS_WARN("Predicted theta & timestamp counter");
	  //printf("Predicted theta: %f \n", pred_theta[m]);  
	  //printf("Timestamp - counter: %f \n", m - FutureTimeStampIndex);
	}
	//printf("Prediction counter: %d \n", m);
	//printf("Future time stamp index: %f \n", FutureTimeStampIndex);
    }
}

Rectangle computeGlobalFreeArea(Rectangle local_freeNavArea, double rw_angle)
{
    Rectangle freeNavArea;
    freeNavArea.x = pred_xy_ropod[j-1].x + local_freeNavArea.x * cos(rw_angle)- local_freeNavArea.y * sin(rw_angle);
    freeNavArea.y = pred_xy_ropod[j-1].y + local_freeNavArea.x * sin(rw_angle)+ local_freeNavArea.y * cos(rw_angle);
    freeNavArea.width = local_freeNavArea.width;
    freeNavArea.depth = local_freeNavArea.depth;
    return freeNavArea;
}



void visualizeDilatedVehicle(std::vector<std::vector<AreaQuad>> footprint_list, int vis_id)
{
	std::vector<AreaQuad> footprint;
	visualization_msgs::Marker vis_dil_Vehicle;
	vis_dil_Vehicle.header.frame_id = "/map";
	vis_dil_Vehicle.header.stamp = ros::Time::now();
	// vis_points.ns = line_strip.ns = line_list.ns = "points_in_map";
	vis_dil_Vehicle.action = visualization_msgs::Marker::ADD;
	vis_dil_Vehicle.pose.orientation.w = 1.0;
	vis_dil_Vehicle.id = vis_id;
	
	vis_dil_Vehicle.color.a = 0.7;
	
	vis_dil_Vehicle.type = visualization_msgs::Marker::LINE_STRIP;
	vis_dil_Vehicle.scale.x = 0.02;
	vis_dil_Vehicle.scale.y = 0.02;
	geometry_msgs::Point area_corner;
	
	vis_dil_Vehicle.points.clear();
	for(unsigned int iSize = 0; iSize < footprint_list.size(); iSize++)
	{
	  vis_dil_Vehicle.color.r = 0.3*iSize;
	  vis_dil_Vehicle.color.g = 0.3*iSize;
	  vis_dil_Vehicle.color.b = 0.3*iSize;
	  
	  footprint = footprint_list[iSize];
	
	  for(unsigned int iPred = 0; iPred < footprint.size()-2; iPred+=3)
	  {
	    area_corner.x = footprint[iPred].p0.x;
	    area_corner.y = footprint[iPred].p0.y;
	    vis_dil_Vehicle.points.push_back(area_corner);
	    area_corner.x = footprint[iPred].p1.x;
	    area_corner.y = footprint[iPred].p1.y;
	    vis_dil_Vehicle.points.push_back(area_corner);
	    area_corner.x = footprint[iPred].p2.x;
	    area_corner.y = footprint[iPred].p2.y;
	    vis_dil_Vehicle.points.push_back(area_corner);
	    area_corner.x = footprint[iPred].p3.x;
	    area_corner.y = footprint[iPred].p3.y;
	    vis_dil_Vehicle.points.push_back(area_corner);
	    area_corner.x = footprint[iPred].p0.x;
	    area_corner.y = footprint[iPred].p0.y;
	    vis_dil_Vehicle.points.push_back(area_corner);
	  
	    freeAreaOR_marker_pub.publish(vis_dil_Vehicle);
	  }
	  footprint.clear();
	}
}
  
void visualizeGlobalFreeArea(Rectangle freeNavArea, double rw_angle, int vis_id)
{
     if(j==1)
    {
        visualization_msgs::Marker vis_free_Area;
        vis_free_Area.header.frame_id = "/map";
        vis_free_Area.header.stamp = ros::Time::now();
        // vis_points.ns = line_strip.ns = line_list.ns = "points_in_map";
        vis_free_Area.action = visualization_msgs::Marker::ADD;
        vis_free_Area.pose.orientation.w = 1.0;
        vis_free_Area.id = vis_id;
        if(vis_id == 100)
        {
          vis_free_Area.color.r = 0.1;
          vis_free_Area.color.g = 0.9;
          vis_free_Area.color.b = 0.1;
        }
        else if(vis_id == 102)
        {
          vis_free_Area.color.r = 0.1;
          vis_free_Area.color.g = 0.1;
	  vis_free_Area.color.b = 0.9;
        }
        else if(vis_id == 104)
	{
	  vis_free_Area.color.r = 0.9;
	  vis_free_Area.color.g = 0.1;
	  vis_free_Area.color.b = 0.9;
	}
        else
	{
	  vis_free_Area.color.r = 0.9;
	  vis_free_Area.color.g = 0.1;
	  vis_free_Area.color.b = 0.1;
	}
        vis_free_Area.color.a = 0.7;

        vis_free_Area.type = visualization_msgs::Marker::LINE_STRIP;
        vis_free_Area.scale.x = 0.02;
        vis_free_Area.scale.y = 0.02;
        geometry_msgs::Point area_corner;

        vis_free_Area.points.clear();
        area_corner.x =  freeNavArea.x + (0.5*freeNavArea.depth*cos(rw_angle) - 0.5*freeNavArea.width*sin(rw_angle));
        area_corner.y =  freeNavArea.y + (0.5*freeNavArea.depth*sin(rw_angle) + 0.5*freeNavArea.width*cos(rw_angle));
        vis_free_Area.points.push_back(area_corner);
        area_corner.x =  freeNavArea.x + (0.5*freeNavArea.depth*cos(rw_angle) + 0.5*freeNavArea.width*sin(rw_angle));
        area_corner.y =  freeNavArea.y + (0.5*freeNavArea.depth*sin(rw_angle) - 0.5*freeNavArea.width*cos(rw_angle));
        vis_free_Area.points.push_back(area_corner);
        area_corner.x =  freeNavArea.x + (-0.5*freeNavArea.depth*cos(rw_angle) + 0.5*freeNavArea.width*sin(rw_angle));
        area_corner.y =  freeNavArea.y + (-0.5*freeNavArea.depth*sin(rw_angle) - 0.5*freeNavArea.width*cos(rw_angle));
        vis_free_Area.points.push_back(area_corner);
        area_corner.x =  freeNavArea.x + (-0.5*freeNavArea.depth*cos(rw_angle) - 0.5*freeNavArea.width*sin(rw_angle));
        area_corner.y =  freeNavArea.y + (-0.5*freeNavArea.depth*sin(rw_angle) + 0.5*freeNavArea.width*cos(rw_angle));
        vis_free_Area.points.push_back(area_corner);
        area_corner.x =  freeNavArea.x + (0.5*freeNavArea.depth*cos(rw_angle) - 0.5*freeNavArea.width*sin(rw_angle));
        area_corner.y =  freeNavArea.y + (0.5*freeNavArea.depth*sin(rw_angle) + 0.5*freeNavArea.width*cos(rw_angle));
        vis_free_Area.points.push_back(area_corner);
	
        freeAreaOR_marker_pub.publish(vis_free_Area);

    }
}

/**
 * Use laser points to create bounding box
 * */
std::vector<bool> laser_point_in_context;
void createFreeNavigationBoundingBox()
{
    double minFreeSpaceDepth = 5; //(config.ROPOD_TO_AX + config.SIZE_FRONT_ROPOD + config.MIN_DIST_TO_OVERTAKE);//+ 3.11*v_obs_sq+1.81);   // config.MIN_DIST_TO_OVERTAKE); // TODO: Add later + ROPOD_LENGTH + config.OBS_AVOID_MARGIN_FRONT so robot fits also while overtaking?
    //printf("Free space depth %f \n", minFreeSpaceDepth);
    double distance_point_to_line_max = 0;
    double distance_point_to_line_min = 100.0;
    double distance_point_to_line;
    no_obs = 0;

    //if(curr_area.type != "hallway")
    //    return;
    rw_p_rear = getPointByID(task1[0],pointlist);
    rw_p_front = getPointByID(task1[1],pointlist);
    double rw_angle = atan2(rw_p_front.y-rw_p_rear.y,rw_p_front.x-rw_p_rear.x);

    double right_lane_space_left = config.TUBE_WIDTH_C;
    double right_lane_space_right = config.TUBE_WIDTH_C;
    double right_lane_space_center_left = 0.5*config.TUBE_WIDTH_C;
    double right_lane_space_center_right = 0.5*config.TUBE_WIDTH_C;
    Rectangle freeNavigationRightLane_R;
    Rectangle freeNavigationRightLane_C;

    double left_lane_space_left = config.TUBE_WIDTH_C;
    double left_lane_space_right = config.TUBE_WIDTH_C;
    double left_lane_space_center_left = 0.5*config.TUBE_WIDTH_C;
    double left_lane_space_center_right = 0.5*config.TUBE_WIDTH_C;
    Rectangle freeNavigationLeftLane_C;
    Rectangle freeNavigationLeftLane_L;

    if (j==1) laser_point_in_context.clear();

    for(unsigned int iScan = 0; iScan < laser_meas_points.size(); iScan++)
    {
        Point laser_point(laser_meas_points[iScan].x, laser_meas_points[iScan].y);
        Point local_robot_wall_laser_point = coordGlobalToRopod(laser_point, pred_xy_ropod[j-1], rw_angle);
        distance_point_to_line = -distToLine(laser_point, rw_p_rear, rw_p_front);
        // consider only points within defined areas
        if (j == 1)
        {
            if ((curr_area.type == "hallway" && isPointOnLeftSide(task1[0], task1[1], pointlist, laser_point, 2.0*config.TUBE_WIDTH_C))
            || ( next_area.type == "hallway" && isPointOnLeftSide(task2[0], task2[1], pointlist, laser_point, 2.0*config.TUBE_WIDTH_C))
            || ( next_second_area.type == "hallway" ) && isPointOnLeftSide(task3[0], task3[1], pointlist, laser_point, 2.0*config.TUBE_WIDTH_C))
            {
                laser_point_in_context.push_back(true);
            }
            else
            {
                laser_point_in_context.push_back(false);
            }         
        }
        if (laser_point_in_context[iScan])
        {
	  //printf("Laser point location %f \n", local_robot_wall_laser_point.x);
	  //printf("Distance point to line %f \n", distance_point_to_line);
            if(distance_point_to_line > 0 && (distance_point_to_line < 2.0*config.TUBE_WIDTH_C) && local_robot_wall_laser_point.x > (-config.ROPOD_TO_AX)  && local_robot_wall_laser_point.x < minFreeSpaceDepth-config.ROPOD_TO_AX)
            {
	      //printf("Right lane space left %f \n", right_lane_space_left);
	      //printf("Right lane space right %f \n", right_lane_space_right);
	      //printf("Right lane space center right %f \n", right_lane_space_center_right);
	      //printf("Right lane space center left %f \n", right_lane_space_center_left);
	      //ROS_WARN("if statement succesfully entered");
	      //printf("Laser point location %f \n", local_robot_wall_laser_point.x);
	      //printf("Distance point to line %f \n", distance_point_to_line);
                // Process right lane
                if(distance_point_to_line < config.TUBE_WIDTH_C)
                {
                    if( (config.TUBE_WIDTH_C - distance_point_to_line) < right_lane_space_left)
                        right_lane_space_left = (config.TUBE_WIDTH_C - distance_point_to_line);
                    if( distance_point_to_line < right_lane_space_right)
                        right_lane_space_right = distance_point_to_line;
                    if( distance_point_to_line < (0.5*config.TUBE_WIDTH_C)  && (0.5*config.TUBE_WIDTH_C - distance_point_to_line) < right_lane_space_center_right )
                        right_lane_space_center_right = (0.5*config.TUBE_WIDTH_C - distance_point_to_line);
                    if( distance_point_to_line >= (0.5*config.TUBE_WIDTH_C) && (distance_point_to_line - 0.5*config.TUBE_WIDTH_C) < right_lane_space_center_left )
                        right_lane_space_center_left = (distance_point_to_line - 0.5*config.TUBE_WIDTH_C);
                }
                 // Process left lane
                else if(distance_point_to_line < 2*config.TUBE_WIDTH_C)
                {
                    if( (2*config.TUBE_WIDTH_C - distance_point_to_line) < left_lane_space_left )
                        left_lane_space_left = (2*config.TUBE_WIDTH_C - distance_point_to_line);
                    if( (distance_point_to_line - config.TUBE_WIDTH_C) < left_lane_space_right )
                        left_lane_space_right = (distance_point_to_line - config.TUBE_WIDTH_C);
                    if( distance_point_to_line < (1.5*config.TUBE_WIDTH_C)  && (1.5*config.TUBE_WIDTH_C - distance_point_to_line) < left_lane_space_center_right )
                        left_lane_space_center_right = (1.5*config.TUBE_WIDTH_C - distance_point_to_line);
                    if( distance_point_to_line >= (1.5*config.TUBE_WIDTH_C) && (distance_point_to_line - 1.5*config.TUBE_WIDTH_C) <   left_lane_space_center_left )
                        left_lane_space_center_left = (distance_point_to_line - 1.5*config.TUBE_WIDTH_C);
                }
            }
        }
    }

    double distance_robot_to_wall = -distToLine(pred_xy_ropod[j-1], rw_p_rear, rw_p_front);

    Rectangle local_freeNavArea;
    local_freeNavArea.width = right_lane_space_right;
    local_freeNavArea.depth = minFreeSpaceDepth;
    local_freeNavArea.x = 0.5*local_freeNavArea.depth-config.ROPOD_TO_AX;
    local_freeNavArea.y = 0.5*local_freeNavArea.width - distance_robot_to_wall;
    freeNavigationRightLane_R = computeGlobalFreeArea(local_freeNavArea, rw_angle);

    local_freeNavArea.width = right_lane_space_center_left + right_lane_space_center_right;
    local_freeNavArea.depth = minFreeSpaceDepth;
    local_freeNavArea.x = 0.5*local_freeNavArea.depth-config.ROPOD_TO_AX;
    local_freeNavArea.y = 0.5*local_freeNavArea.width - distance_robot_to_wall + (0.5*config.TUBE_WIDTH_C - right_lane_space_center_right);
    freeNavigationRightLane_C = computeGlobalFreeArea(local_freeNavArea, rw_angle);

    if (freeNavigationRightLane_R.width >= freeNavigationRightLane_C.width)
    {
        freeNavigationRightLaneRight = freeNavigationRightLane_R;
    }
    else
    {
        freeNavigationRightLaneRight = freeNavigationRightLane_C;
    }

    visualizeGlobalFreeArea(freeNavigationRightLaneRight, rw_angle, 100);

    local_freeNavArea.width = fmin(right_lane_space_left + left_lane_space_right, config.TUBE_WIDTH_C);
    local_freeNavArea.depth = minFreeSpaceDepth;
    local_freeNavArea.x = 0.5*local_freeNavArea.depth-config.ROPOD_TO_AX;
    local_freeNavArea.y = 0.5*local_freeNavArea.width - distance_robot_to_wall + (config.TUBE_WIDTH_C - right_lane_space_left);
    freeNavigationCenter = computeGlobalFreeArea(local_freeNavArea, rw_angle);
    //visualizeGlobalFreeArea(freeNavigationCenter, rw_angle, 101);

    local_freeNavArea.width = left_lane_space_center_left + left_lane_space_center_right;
    local_freeNavArea.depth = minFreeSpaceDepth;
    local_freeNavArea.x = 0.5*local_freeNavArea.depth-config.ROPOD_TO_AX;
    local_freeNavArea.y = 0.5*local_freeNavArea.width - distance_robot_to_wall + (1.5*config.TUBE_WIDTH_C - left_lane_space_center_right);
    freeNavigationLeftLane_C = computeGlobalFreeArea(local_freeNavArea, rw_angle);
    //visualizeGlobalFreeArea(freeNavigationLeftLane_C, rw_angle, 102);

    local_freeNavArea.width = left_lane_space_left;
    local_freeNavArea.depth = minFreeSpaceDepth;
    local_freeNavArea.x = 0.5*local_freeNavArea.depth-config.ROPOD_TO_AX;
    local_freeNavArea.y = 0.5*local_freeNavArea.width - distance_robot_to_wall + (2.0*config.TUBE_WIDTH_C - left_lane_space_left);
    freeNavigationLeftLane_L = computeGlobalFreeArea(local_freeNavArea, rw_angle);
    //visualizeGlobalFreeArea(freeNavigationLeftLane_L, rw_angle, 103);

    if ( freeNavigationLeftLane_C.width >= freeNavigationLeftLane_L.width)
        freeNavigationLeftLaneLeft = freeNavigationLeftLane_C;
    else
        freeNavigationLeftLaneLeft = freeNavigationLeftLane_L;

    visualizeGlobalFreeArea(freeNavigationLeftLaneLeft, rw_angle, 104);
}

void considerOvertaking()
{
    // Consider overtaking.
    // TODO: Add freeNavigationLeftLaneLeft;
    // TODO: This function might not be necessary
    overtake_on_current_hallway = false;
    if (u < ka_max-1) 
    {
        if (curr_area.type == "hallway")
	{
            current_hallway = curr_area;
            current_hallway_task = task1;
            overtake_on_current_hallway = true;
        }
    }
}

void overtakeStateMachine()
{
    // TODO: filter fastest obstacle in front of the robot
    v_obs_sq = current_obstacle.rectangle.vel.x*current_obstacle.rectangle.vel.x+current_obstacle.rectangle.vel.y*current_obstacle.rectangle.vel.y;
    if ((v_obs_sq < config.V_OBS_OVERTAKE_MAX*config.V_OBS_OVERTAKE_MAX) && overtake_on_current_hallway) 
    {
        rw_p_rear = getPointByID(current_hallway_task[0],pointlist);
        rw_p_front = getPointByID(current_hallway_task[1],pointlist);
        cur_obj = current_hallway;
        areaIDs = cur_obj.getPointIDs();
        ind = 0;
        qmax = areaIDs.size();
        for (int q = 0; q < qmax; ++q) 
	{
            if (areaIDs[q].compare(rw_p_rear.id) == 0) 
	    {
                ind = q;
            }
        }
        rotate(areaIDs.begin(), areaIDs.begin() + ind, areaIDs.end());
        lw_p_rear = getPointByID(areaIDs[3],pointlist);
        lw_p_front = getPointByID(areaIDs[2],pointlist);
        // TODO: Add freeNavigationLeftLaneLeft;

        if (freeNavigationRightLaneRight.width >= config.TUBE_WIDTH_C) 
	{
            if (j == 1) ROS_INFO("No overtake necessary, passing on right should be possible");
	    //printf("RightlaneRight width %f \n", freeNavigationRightLaneRight.width);
	    //printf("RightlaneRight depth %f \n", freeNavigationRightLaneRight.depth);
	    //printf("Tube width %f \n", config.TUBE_WIDTH_C);
            shift_wall = 0;
        } 
        else if (freeNavigationRightLaneRight.width > 2*(config.SIZE_SIDE+config.OBS_AVOID_MARGIN)) 
	{
            // Same state, but change tube width so ropod will
            // fit through space right
            Point wall_pos(freeNavigationRightLaneRight.x, freeNavigationRightLaneRight.y);
            double distAreatoWall = -distToLine(wall_pos, rw_p_rear, rw_p_front);
            shift_wall = distAreatoWall - freeNavigationRightLaneRight.width/2;
            pred_tube_width[j] = freeNavigationRightLaneRight.width;
            if (j == 1) ROS_INFO("No overtake necessary, but tube size scaled down");
        } 
        else if (freeNavigationLeftLaneLeft.width > 2*(config.SIZE_SIDE+config.OBS_AVOID_MARGIN)) {
            if (j == 1) ROS_INFO("Can overtake on left side, there should be enough space there");
            // Start overtake
            if (freeNavigationLeftLaneLeft.width < 2*(config.SIZE_SIDE+config.ENV_TRNS_SIZE)) 
	    {
                if (j == 1) ROS_INFO("Tight overtake");
                pred_state[j] = TIGHT_OVERTAKE;
            } 
            else 
	    {
                if (j == 1) ROS_INFO("Spacious overtake");
                pred_state[j] = SPACIOUS_OVERTAKE;
                // TODO: for now obstacle angle is aligned with wall (from bounding box) So only width is looked at
            }
            Point wall_pos(freeNavigationLeftLaneLeft.x, freeNavigationLeftLaneLeft.y);
            double distAreatoWall = -distToLine(wall_pos, rw_p_rear, rw_p_front);
            shift_wall = distAreatoWall - freeNavigationLeftLaneLeft.width/2 + config.OBS_AVOID_MARGIN;
            pred_tube_width[j] = freeNavigationLeftLaneLeft.width;
        } 
        else 
	{
            if (j == 1) ROS_INFO("No overtake possible, stuck behind this obstacle");
            shift_wall = 0;
        }
    }
}

/**
 * Check for collision
 * Either for obstacles or virtual walls(to be added)
 * */
void checkForCollisions()
{
    if (no_obs > 0) 
    {
        pred_x_obs[j] = pred_x_obs[j-1]+current_obstacle.rectangle.vel.x*TS*F_FSTR;
        pred_y_obs[j] = pred_y_obs[j-1]+current_obstacle.rectangle.vel.y*TS*F_FSTR;
    }

    // Dilated vehicle
    pred_ropod_dil_rb.x = pred_x_rearax[m] + (config.SIZE_REAR+(config.OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]+M_PI) + (config.SIZE_SIDE+(config.OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]-M_PI/2);
    pred_ropod_dil_rb.y = pred_y_rearax[m] + (config.SIZE_REAR+(config.OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]+M_PI) + (config.SIZE_SIDE+(config.OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]-M_PI/2);
    pred_ropod_dil_lb.x = pred_x_rearax[m] + (config.SIZE_REAR+(config.OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]+M_PI) + (config.SIZE_SIDE+(config.OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]+M_PI/2);
    pred_ropod_dil_lb.y = pred_y_rearax[m] + (config.SIZE_REAR+(config.OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]+M_PI) + (config.SIZE_SIDE+(config.OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]+M_PI/2);
    pred_ropod_dil_lt.x = pred_x_rearax[m] + (config.SIZE_FRONT_RAX+(config.OBS_AVOID_MARGIN_FRONT*v_scale))*cos(pred_theta[m]) + (config.SIZE_SIDE+(config.OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]+M_PI/2);
    pred_ropod_dil_lt.y = pred_y_rearax[m] + (config.SIZE_FRONT_RAX+(config.OBS_AVOID_MARGIN_FRONT*v_scale))*sin(pred_theta[m]) + (config.SIZE_SIDE+(config.OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]+M_PI/2);
    pred_ropod_dil_rt.x = pred_x_rearax[m] + (config.SIZE_FRONT_RAX+(config.OBS_AVOID_MARGIN_FRONT*v_scale))*cos(pred_theta[m]) + (config.SIZE_SIDE+(config.OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]-M_PI/2);
    pred_ropod_dil_rt.y = pred_y_rearax[m] + (config.SIZE_FRONT_RAX+(config.OBS_AVOID_MARGIN_FRONT*v_scale))*sin(pred_theta[m]) + (config.SIZE_SIDE+(config.OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]-M_PI/2);
    AreaQuad robot_footprint(pred_ropod_dil_rb, pred_ropod_dil_lb, pred_ropod_dil_lt, pred_ropod_dil_rt);
    robot_footprint_list.push_back(robot_footprint);

    ropod_colliding_obs = false;
    pred_ropod_colliding_obs[j] = false;
    // Obstacle detection (crappy implementation in C++)
    //if (t_pred[m] < config.T_PRED_OBS_COLLISION && no_obs > 0) {
        //obslt.x = pred_x_obs[j]+current_obstacle.rectangle.width/2*cos(obs_theta)-current_obstacle.rectangle.depth/2*sin(obs_theta);
        //obslt.y = pred_y_obs[j]+current_obstacle.rectangle.width/2*sin(obs_theta)+current_obstacle.rectangle.depth/2*cos(obs_theta);
        //obsrt.x = pred_x_obs[j]+current_obstacle.rectangle.width/2*cos(obs_theta)+current_obstacle.rectangle.depth/2*sin(obs_theta);
        //obsrt.y = pred_y_obs[j]+current_obstacle.rectangle.width/2*sin(obs_theta)-current_obstacle.rectangle.depth/2*cos(obs_theta);
        //obslb.x = pred_x_obs[j]-current_obstacle.rectangle.width/2*cos(obs_theta)-current_obstacle.rectangle.depth/2*sin(obs_theta);
        //obslb.y = pred_y_obs[j]-current_obstacle.rectangle.width/2*sin(obs_theta)+current_obstacle.rectangle.depth/2*cos(obs_theta);
        //obsrb.x = pred_x_obs[j]-current_obstacle.rectangle.width/2*cos(obs_theta)+current_obstacle.rectangle.depth/2*sin(obs_theta);
        //obsrb.y = pred_y_obs[j]-current_obstacle.rectangle.width/2*sin(obs_theta)-current_obstacle.rectangle.depth/2*cos(obs_theta);
        //pred_ropod_colliding_obs[j] = do_shapes_overlap(pred_ropod_dil_rb, pred_ropod_dil_lb, pred_ropod_dil_lt, pred_ropod_dil_rt, obsrt, obslt, obslb, obsrb);
        //ropod_colliding_obs = pred_ropod_colliding_obs[j];
    //}

    // TODO: Add here also check with raw laser (for instance of non-associated objects) data or costmap
    if (t_pred[m] < config.T_PRED_OBS_COLLISION ) // TODO: Condition should be smarter. For instance based on time/distance to stop? . Also the robot should stop at a minimum distance in front. for now we based it on time
    {
        // Here check for collision with laser data
        // Create Area and use contain method with the laser data.

        for(unsigned int iScan = 0; iScan < laser_meas_points.size(); iScan++)
        {
            Point laser_point(laser_meas_points[iScan].x, laser_meas_points[iScan].y);
            pred_ropod_colliding_obs[j] = robot_footprint.contains(laser_point);
            ropod_colliding_obs = pred_ropod_colliding_obs[j];
            if(ropod_colliding_obs)
                break;
        }
    }
    // Predict intersection times (left out for now)
    // TODO: Finish next line to add checking collision with walls!
    // Predict intersection with walls
    if (t_pred[m] < config.T_PRED_WALL_COLLISION) 
    {
        rw_p_rear = getPointByID(task1[0],pointlist);
        Point rw_p_rear_noid(rw_p_rear.x,rw_p_rear.y);
        rw_p_front = getPointByID(task1[1],pointlist);
        Point rw_p_front_noid(rw_p_front.x,rw_p_front.y);
        double distance_point_to_line = -distToLine(pred_xy_ropod[j-1], rw_p_rear, rw_p_front);
        bool ropod_intersect_wall = does_line_intersect_shape(rw_p_rear_noid, rw_p_front_noid, robot_footprint);
        if (distance_point_to_line > 0 && ropod_intersect_wall == false)
        {
            robot_left_side_wall_first_time = true;
        }
        if (distance_point_to_line > 0 && robot_left_side_wall_first_time) // if ropod is at the right side of the wall and ropod did not start colliding virtual wall (allow to recover)
	{ 
            ropod_colliding_wall = does_line_intersect_shape(rw_p_rear_noid, rw_p_front_noid, robot_footprint);
        }
     }
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

void getDebugRoutePlanCallback(const ropod_ros_msgs::RoutePlannerResultConstPtr& result)
{
    debug_route_planner_result_ = *result;

    ROS_INFO("new debug plan received");
    start_navigation = true;
}

void resetNavigation()
{
    assignment.clear();
    arealist.clear();
    pointlist.clear();
    arealist.clear();
    OBJ_X_TASK.clear();
    task1.clear();
    task2.clear();
    task3.clear();
    current_hallway_task.clear();
    next_hallway_task.clear();
    ropod_reached_target = false;
    start_navigation = false;

    i = 0; // - simulation/experiment plan
    n = 1; // - simulation/experiment movement
    k = -1; //- velocity scaling
    q = 1; // - for loop for movement simulation (in prediction)
    j = 0; // - prediction plan
    m = 0; // - prediction movement
    u = 0; // - Pred task counter

    t_pred[0] = 0;              // Prediction time [s]
    t_pred_j[0] = 0;              // Prediction time planning [s]
    pred_v_ropod[0] = v_ropod_0;
    pred_v_ropod_plan[0] = pred_v_ropod[0];
    pred_ropod_on_entry_inter[0] = false;
    pred_ropod_on_entry_hall[0] = false;
    pred_x_ropod[0] = x_ropod_0;
    pred_y_ropod[0] = y_ropod_0;
    pred_x_obs[0] = 0;
    pred_y_obs[0] = 0;
    v_obs_sq = 0;
    pred_plan_theta[0] = theta_0;
    pred_accel[0] = 0;
    sim_theta[0] = pred_plan_theta[0];
    sim_phi[0] = phi_0;
    sim_v_scale[0] = 1;
    prev_sim_tube_width = config.TUBE_WIDTH_C;
    pred_phi_des[0] = 0;
    pred_tube_width[0] = config.TUBE_WIDTH_C;
    v_des_scaled[0] = 0;
    pred_ropod_colliding_obs[0] = false;

    prev_sim_state = 1;
    pred_state[0] = 1;
    prev_sim_task_counter = 0;
    pred_task_counter[0] = 0;
}

void followRoute(std::vector<ropod_ros_msgs::Area> planner_areas,
                 ros::Publisher& vel_pub, ros::Rate& rate)
{
    resetNavigation();

    ROS_INFO("Now preparing the plan");
    initializeVisualizationMarkers();

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
                vis_plan.id = 1000*i+j;
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
    start_navigation = true;


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

    while(ros::ok() && !ropod_reached_target)
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
 /*       if (this_amcl_x == prev_amcl_x && this_amcl_y == prev_amcl_y && this_amcl_theta == prev_amcl_theta) {
            // No AMCL update, so estimate initial values for prediction
            ropod_x = pred_x_ropod[1];
            ropod_y = pred_y_ropod[1];
            ropod_theta = pred_plan_theta[1];
        } else {
        */
        //TODO: Change to time check! only predict durin 1/3 ec, otherwise reset to las AMCL!
            // Set latest values from AMCL to previous AMCL values for next iteration
            // And take AMCL values as initial for the prediction
            // prev_amcl_x = this_amcl_x;
            // prev_amcl_y = this_amcl_y;
            // prev_amcl_theta = this_amcl_theta;
            // ropod_x = this_amcl_x;
            // ropod_y = this_amcl_y;
            // ropod_theta = this_amcl_theta;
      //  }

        //ROS_INFO("Ropod x: %f / Ropod y: %f / Theta: %f", ropod_x, ropod_y, ropod_theta);
        //ROS_INFO("xdot: %f / ydot: %f / thetadot %f", odom_xdot_ropod_global, odom_ydot_ropod_global, odom_thetadot_global);
        //ROS_INFO("ropodx: %f / ropody: %f / ropodtheta %f", ropod_x, ropod_y, ropod_theta);

        try
        {
            tf::StampedTransform t_ropod_pose;
            tf_listener_->lookupTransform("map", "/ropod/base_link", ros::Time(0), t_ropod_pose);

            ropod_x = t_ropod_pose.getOrigin().getX();
            ropod_y = t_ropod_pose.getOrigin().getY();
            ropod_theta = tf::getYaw(t_ropod_pose.getRotation());

        }
        catch(tf::TransformException ex)
        {
            ROS_ERROR("Error while getting ropod pose");
        }
	std::vector<std::vector<AreaQuad>> prediction_list;
	
        while ((ropod_colliding_obs || ropod_colliding_wall) && k < V_SCALE_OPTIONS.size())
        {
            v_scale = V_SCALE_OPTIONS[k];
	    printf("Velocity scale: %f", v_scale);
            k++;
            m = 0;
            j = 0;
            t_pred[0] = 0;
            t_pred_j[0] = 0;

            //ROS_INFO("xdot: %f \t ydot: %f", odom_xdot_ropod_global, odom_ydot_ropod_global);

            // Initialize prediction with latest sim values
            pred_phi[0] = odom_phi_local; // On ropod
            pred_theta[0] = ropod_theta;
            pred_v_ropod[0] = control_v;
            if (abs(odom_vropod_global-abs(control_v)) > 0.5) {
	      ROS_INFO("Difference between control and actual velocity > 0.5, correcting now.");
              pred_v_ropod[0] = odom_vropod_global;
            }
            pred_xdot[0] = pred_v_ropod[0]*cos(pred_phi[0])*cos(ropod_theta);   // xdot of rearaxle in global frame
            pred_ydot[0] = pred_v_ropod[0]*cos(pred_phi[0])*sin(ropod_theta);   // ydot of rearaxle in global frame
            pred_thetadot[0] = pred_v_ropod[0]*1/config.D_AX*sin(pred_phi[0]);
            pred_x_rearax[0] = ropod_x-config.D_AX*cos(ropod_theta);
            pred_y_rearax[0] = ropod_y-config.D_AX*sin(ropod_theta);
            pred_xy_ropod[0].x = ropod_x;
            pred_xy_ropod[0].y = ropod_y;
            prev_pred_phi_des = prev_sim_phi_des;
            pred_phi_des[0] = prev_pred_phi_des;
            pred_tube_width[0] = prev_sim_tube_width;
            pred_plan_theta[0] = ropod_theta;
            pred_v_ropod_plan[0] = pred_v_ropod[0];
            pred_state[0] = prev_sim_state;
            pred_task_counter[0] = prev_sim_task_counter;
            pred_x_obs[0] = current_obstacle.rectangle.pose.position.x;
            pred_y_obs[0] = current_obstacle.rectangle.pose.position.y;

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
            while (t_pred[m] < config.T_MIN_PRED) {
                j = j+1;

                // j and m counter are initialized at 0 instead of 1 in Matlab so we dont have to change their indices
                robot_left_side_wall_first_time = false;
                ropod_colliding_wall =  false;

                overtake_on_current_hallway = false;
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
                    point_rear = getPointByID(task1[0],pointlist);
                    local_wallpoint_front = coordGlobalToRopod(point_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    double distance_to_end_sqrd = distToEndSegmentSquared(pred_xy_ropod[j-1], point_rear, point_front);
                    if(local_wallpoint_front.x < config.ENTRY_LENGTH && distance_to_end_sqrd <= config.ENTRY_LENGTH*config.ENTRY_LENGTH )
                        pred_ropod_on_entry_inter[j] = true;
                    else
                        pred_ropod_on_entry_inter[j] = false;

                }
                else if (area1ID!=area2ID && curr_area.type == "hallway" && next_area.type == "hallway")
                {
                    double walls_angle = getAngleBetweenHallways(task1, task2, pointlist);
                    double distance_to_switch_halls;
                    if(walls_angle > 0) // Concave, switch early
                        distance_to_switch_halls = config.SIZE_FRONT_ROPOD+1.0;
                    else // Convex, switch close to turning axis
                        distance_to_switch_halls = -0.5*config.D_AX;

                    point_front = getPointByID(task1[1],pointlist);
                    point_rear = getPointByID(task1[0],pointlist);
                    local_wallpoint_front = coordGlobalToRopod(point_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    double distance_to_end_sqrd = distToEndSegmentSquared(pred_xy_ropod[j-1], point_rear, point_front);
                    if(local_wallpoint_front.x < distance_to_switch_halls && distance_to_end_sqrd <= distance_to_switch_halls*distance_to_switch_halls)
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
                createFreeNavigationBoundingBox();
                considerOvertaking();

                /**
                 * Call overtake state machine when needed
                 * */
                if (overtake_on_current_hallway)
                {
                    overtakeStateMachine();
                }
                else // TODO: check if this is necessary
                {
                    pred_tube_width[j] = config.TUBE_WIDTH_C;
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
		
                if(ropod_colliding_obs || ropod_colliding_wall){ // Do not execute current plan when collision is predicted
		    //printf("Colliding with wall? %d \n", ropod_colliding_wall);
		    //printf("Colliding with obs? %d \n", ropod_colliding_obs);
                    break;
		}


                visualizeRopodMarkers();

                // Update positions used to make the prediction plan with
                // Cesar-> TODO: In theory D_AX should be replaced by ROPOD_TO_AX, but it brings issues.
                //               Or rename predictions not to ropod but to steering point?
                pred_x_ropod[j] = pred_x_rearax[m]+config.D_AX*cos(pred_theta[m]);
                pred_y_ropod[j] = pred_y_rearax[m]+config.D_AX*sin(pred_theta[m]);
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
            
	    prediction_list.push_back(robot_footprint_list);
	    robot_footprint_list.clear();
	    //printf("k: %d", k);
        }          // end while finding v_scale
	visualizeDilatedVehicle(prediction_list, 200);
	prediction_list.clear();

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

        if(ropod_colliding_obs || ropod_colliding_wall)
        {
            control_v = 0.0; // Force stop if even smallest scale failed
        }
        else
        {
            control_v = pred_v_ropod[0]+pred_accel[1]*1/F_PLANNER;
        }
        // Compute v_ax and theta_dot from v_des and phi

        myfile << real_time_est << "\t" << program_duration << "\t" << pred_state[0] << "\t" << pred_task_counter[0] << "\t" << pred_tube_width[0] << "\t" <<
            pred_phi_des[0] << "\t" << pred_v_ropod[0] << "\t" << control_v << "\t"  <<  pred_xy_ropod[0].x << "\t" <<  pred_xy_ropod[0].y << "\t" <<
            pred_plan_theta[0] << "\t" << pred_x_obs[0] << "\t" << pred_y_obs[0] << "\t" << obs_theta << "\t" << current_obstacle.rectangle.width << "\t" <<
            current_obstacle.rectangle.depth << "\t" << current_obstacle.rectangle.vel.x << "\t" << current_obstacle.rectangle.vel.y << "\t" << pred_accel[1] << "\t" <<"\n";
        /*
        //ROS_INFO("Phi: %f / V_ax: %f / Theta_dot: %f", pred_phi_des[1], v_ax, theta_dot);
        ROS_INFO("state: %d, tube width: %f", pred_state[1], pred_tube_width[1]);
        ROS_INFO("K: %d", k);
        ROS_INFO("V desired: %f", pred_v_ropod_plan[1]);
        ROS_INFO("Predphi[1]: %f / [2]: %f / [3]: %f / [4]: %f", pred_phi_des[1], pred_phi_des[2], pred_phi_des[3], pred_phi_des[4]);
        */

        // if (pred_v_ropod_plan[1] > 0) {
        //     v_ax = cos(pred_phi_des[1])*pred_v_ropod_plan[1];
        //     theta_dot = pred_v_ropod_plan[1]/config.D_AX*sin(pred_phi_des[1]);
        //     publishCustomVelocity(v_ax, theta_dot);
        // } else {
        //     publishZeroVelocity();
        // }
        if (control_v > 0) {
	    printf("Control velocity %f \n", control_v);
            v_ax = cos(pred_phi_des[1])*control_v;
            theta_dot = control_v/config.D_AX*sin(pred_phi_des[1]);
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = v_ax;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = theta_dot;
            vel_pub.publish(cmd_vel);
        } else {
	    v_ax = cos(pred_phi_des[1])*control_v;
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = v_ax;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            vel_pub.publish(cmd_vel);
        }
        if (prev_sim_task_counter == (ka_max-1) ) {

            point_rear = getPointByID(task1[0],pointlist);
            point_front = getPointByID(task1[1],pointlist);
            local_wallpoint_front = coordGlobalToRopod(point_front, pred_xy_ropod[0], pred_plan_theta[1]);
            double distance_to_end_sqrd = distToEndSegmentSquared(pred_xy_ropod[0], point_rear, point_front);
            if( local_wallpoint_front.x < config.REACHEDTARGETTHRESHOLD && distance_to_end_sqrd < config.REACHEDTARGETTHRESHOLD*config.REACHEDTARGETTHRESHOLD)
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
        x_rearax = ropod_x - config.D_AX*cos(ropod_theta); // X position of center of rear axle [m]
        y_rearax = ropod_y - config.D_AX*sin(ropod_theta); // Y position of center of rear axle [m]
        vis_rt.x = x_rearax+(config.D_AX+config.SIZE_FRONT_ROPOD)*cos(ropod_theta)+config.SIZE_SIDE*cos(ropod_theta-0.5*M_PI);
        vis_rt.y = y_rearax+(config.D_AX+config.SIZE_FRONT_ROPOD)*sin(ropod_theta)+config.SIZE_SIDE*sin(ropod_theta-0.5*M_PI);
        vis_lt.x = x_rearax+(config.D_AX+config.SIZE_FRONT_ROPOD)*cos(ropod_theta)+config.SIZE_SIDE*cos(ropod_theta+0.5*M_PI);
        vis_lt.y = y_rearax+(config.D_AX+config.SIZE_FRONT_ROPOD)*sin(ropod_theta)+config.SIZE_SIDE*sin(ropod_theta+0.5*M_PI);
        vis_fr.x = config.FEELER_SIZE_STEERING*cos(ropod_theta+prev_sim_phi_des);
        vis_fr.y = config.FEELER_SIZE_STEERING*sin(ropod_theta+prev_sim_phi_des);
        vis_fl.x = config.FEELER_SIZE_STEERING*cos(ropod_theta+prev_sim_phi_des);
        vis_fl.y = config.FEELER_SIZE_STEERING*sin(ropod_theta+prev_sim_phi_des);
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
            ROS_WARN("Control loop missed its desired rate of %.4f Hz... the loop actually took %.4f seconds", F_PLANNER, rate.cycleTime().toSec());
        }

    }
    // Will only perform this when ropod has reached target
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub.publish(cmd_vel);
    myfile.close();
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
    ros::Publisher vel_pub_;
    ros::Rate napoleon_rate_;

public:
    NapoleonPlanner(std::string name, ros::Publisher& vel_pub, ros::Rate& napoleon_rate) :
    as_(nh_, name, boost::bind(&NapoleonPlanner::executeCB, this, _1), false),
    action_name_(name), ac_("/route_planner", true), status_(false), napoleon_rate_(napoleon_rate)
    {
        vel_pub_ = vel_pub;
    }

    bool start()
    {
        ROS_INFO("Waiting for route planner action server to start");
        if(ac_.waitForServer( ros::Duration(5.0)))
        {
            ROS_INFO("Connected to route planner action server");
            // waiting for route planner action server to start
            as_.start();
            ROS_INFO("Waiting for GOTO action");
            return true;
        }
        else
        {
            ROS_INFO("Server not started. Navigation still available via debug topic");
            return false;
        }
    }

    ~NapoleonPlanner(void)
    {
    }

    bool getStatus()
    {
        return status_;
    }

    void setStatus(bool status)
    {
        status_ = status;
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
        status_ = false;
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

        if (status_)
        {
            std::vector<ropod_ros_msgs::Area> planner_areas = route_planner_result_.areas;

            ROS_INFO("Got new route; following now");
            followRoute(planner_areas, vel_pub_, napoleon_rate_);

            status_ = false;
            as_.setSucceeded();
        }
    }
};
NapoleonPlanner* napoleon_planner;

int main(int argc, char** argv)
{

    ros::init(argc, argv, "route_navigation");
    ros::NodeHandle nroshndl("~");
    ros::Rate rate(F_PLANNER);

    ros::Subscriber goal_cmd_sub = nroshndl.subscribe<geometry_msgs::PoseStamped>("/route_navigation/simple_goal", 10, simpleGoalCallback);
    ros::Subscriber amcl_pose_sub = nroshndl.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, getAmclPoseCallback);
    ros::Subscriber ropod_odom_sub = nroshndl.subscribe<nav_msgs::Odometry>("/ropod/odom", 100, getOdomVelCallback);
    ros::Subscriber ropod_debug_plan_sub = nroshndl.subscribe<ropod_ros_msgs::RoutePlannerResult>("/ropod/debug_route_plan", 1, getDebugRoutePlanCallback);
    ros::Subscriber FutureTimeStamp = nroshndl.subscribe<std_msgs::Float64>("/MO/future_time_stamp", 1, getFutureTimeStampCallback);

    ros::Subscriber obstacle_sub = nroshndl.subscribe<ed_gui_server::objsPosVel>("/ed/gui/objectPosVel", 10, getObstaclesCallback);
    ros::Publisher vel_pub = nroshndl.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    fut_phi_pub = nroshndl.advertise<std_msgs::Float64>("/MO/fut_phi", 1, true);
    pred_pose_vel_pub = nroshndl.advertise<nav_msgs::Odometry>("/ropod/pred_pose_vel", 1, true);
    
    // Visualize map nodes and robot
    ropodmarker_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_driving/ropodpoints", 1);
    mapmarker_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_driving/wmnodes", 100, true);
    freeAreaOR_marker_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_driving/freeAreaOvertakeRight", 10, true);
    freeAreaOL_marker_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_driving/freeAreaOvertakeLeft", 10, true);
    wallmarker_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_driving/right_side_wall", 10, true);
    pivotmarker_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_driving/pivot", 10, true);
    tf_listener_ = new tf::TransformListener;
    // Subscribe to topic with non-associated laser points (for now all laser points)
    unsigned int bufferSize = 2;
    ros::Subscriber scan_sub = nroshndl.subscribe<sensor_msgs::LaserScan>("scan", bufferSize, scanCallback);

    dynamic_reconfigure::Server<napoleon_navigation::NapoleonNavigationConfig> dyn_recon_srv;
    dynamic_reconfigure::Server<napoleon_navigation::NapoleonNavigationConfig>::CallbackType f;
    f = boost::bind(&dynamicReconfigureCallback, _1, _2);
    dyn_recon_srv.setCallback(f);

    napoleon_planner = new NapoleonPlanner("/napoleon/goto", vel_pub, rate);
    action_server_enabled = napoleon_planner->start();
    if(action_server_enabled) ROS_INFO("Wait for goto action");
    while(nroshndl.ok())
    {
        if (action_server_enabled)
        {
            if (napoleon_planner->getStatus())
            {
                ROS_INFO("Received goto action");
            }

            while (napoleon_planner->getStatus())
            {
                ros::spinOnce();
            }
        }

        if(start_navigation)
        {
            std::vector<ropod_ros_msgs::Area> planner_areas = debug_route_planner_result_.areas;
            ROS_INFO("Got new route via debug topic; following now");
            followRoute(planner_areas, vel_pub, rate);
            start_navigation =  false;
        }
        ros::spinOnce();
    }

    return 0;
}
