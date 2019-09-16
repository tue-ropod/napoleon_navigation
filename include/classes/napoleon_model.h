#ifndef NAP_MOD_H
#define NAP_MOD_H

#include <queue>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "napoleon_config.h"
#include "napoleon_prediction.h"
#include "napoleon_obstacle.h"
#include "napoleon_visualization.h"

class NapoleonModel {

public:

    tf::TransformListener *tf_listener_;

    std::vector<geometry_msgs::Point> laser_meas_points;
    sensor_msgs::LaserScan::ConstPtr scan;
    std::queue<sensor_msgs::LaserScan::ConstPtr> scan_buffer_;
    bool scan_available = false;

    double ropod_x = 0, ropod_y = 0, ropod_theta = 0;
    double this_amcl_x = 0, this_amcl_y = 0, quaternion_x = 0, quaternion_y = 0, quaternion_z = 0, quaternion_w = 0, this_amcl_theta = 0, siny_cosp = 0, cosy_cosp = 0;
    double odom_xdot_ropod_global = 0, odom_ydot_ropod_global = 0, odom_thetadot_global = 0, odom_phi_local = 0, odom_phi_global = 0, odom_vropod_global = 0;

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

    double v_ax = 0, theta_dot = 0, v_des, phi, v_scale;

    bool ropod_colliding_obs = true;     // Initially set to true
    bool ropod_colliding_wall = true;    // Initially set to true

    bool consider_overtaking_current_hallway, consider_overtaking_next_hallway;

    double delta_t = 1/(double)F_PLANNER;           // Time ropod will execute this plan
    double max_delta_v = A_MAX*delta_t;             // Maximum change in v in delta_t

    // State of ropod
    enum {
        CRUSING = 1,
        ENTRY_BEFORE_TURN_ON_INTERSECTION,
        ACCELERATE_ON_INTERSECTION,
        ALIGN_AXIS_AT_INTERSECTION, //  (& slow down)
        TURNING,
        ENTRY_BEFORE_GOING_STRAIGHT_ON_INTERSECTION,
        GOING_STRAIGHT_ON_INTERSECTION,
        TIGHT_OVERTAKE,     // (follow left wall)
        SPACIOUS_OVERTAKE   // (shift right wall to left virtually)
    };

public:

    NapoleonModel() {
        tf_listener_ = new tf::TransformListener;
    }

    void getOdomVelCallback(const nav_msgs::Odometry::ConstPtr &odom_vel);
    void getAmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void getLatestScanData();
    void updatePosition(NapoleonPrediction &P);
    void updateControlVelocity(NapoleonPrediction &P);
    void overtakeStateMachine(NapoleonPrediction &P, NapoleonObstacle &O, NapoleonAssignment &A);
    void computeSteeringAndVelocity(ros::Publisher &wallmarker_pub, NapoleonAssignment &A, NapoleonPrediction &P, NapoleonVisualization &V);

};

#endif
