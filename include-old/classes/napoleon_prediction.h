#ifndef NAP_PRE_H
#define NAP_PRE_H

#include <array>
#include <ros/ros.h>
#include "napoleon_geometry.h"
#include "napoleon_config.h"
#include "napoleon_functions.h"

#include "napoleon_obstacle.h"
#include "napoleon_model.h"
#include "napoleon_assignment.h"
#include "napoleon_visualization.h"

class NapoleonAssignment;
class NapoleonModel;
class NapoleonObstacle;
class NapoleonVisualization;

using namespace std;

class NapoleonPrediction{

public:

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

    static constexpr int size_m = ((int)F_MODEL)*(T_MAX_PRED+1)+1;  // Array size for predictions that run at F_MODEL
    static constexpr int size_p = ((int)F_PLANNER)*(T_MAX_PRED+1)+1;   // Array size for predictions that run at F_PLANNER

    int prev_sim_state = 1;
    int pred_state[size_p] {1};
    int prev_sim_task_counter = 0;
    int pred_task_counter[size_p] {0};
    int danger_count = 0; // Counter that ensures program stops if collision with wall predicted for sometime

    bool pred_ropod_on_entry_inter[size_p] {false};
    bool pred_ropod_on_entry_hall[size_p] {false};

    double t_pred[size_m] {0};              // Prediction time [s]
    double t_pred_j[size_p] {0};              // Prediction time planning [s]

    double pred_v_ropod[size_m];
    double pred_v_ropod_plan[size_p];
    double pred_x_ropod[size_p];
    double pred_y_ropod[size_p];
    double pred_plan_theta[size_p];
    std::vector<double> sim_theta;
    std::vector<double> sim_phi;
    Point pred_xy_ropod_0;
    std::array<Point, size_p> pred_xy_ropod;

    double pred_x_obs[size_p] {0};
    double pred_y_obs[size_p] {0};
    double prev_sim_phi_des;
    double pred_accel[size_p] {0};

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
    bool update_state_points;
    bool dir_cw;
    double v_des_scaled[size_p] {0};
    double vel_dif, v_new;
    double t_pred_prev;
    double dist_to_middle_final;
    bool pred_ropod_colliding_obs[size_p] {false};
    bool consider_overtaking_current_hallway, consider_overtaking_next_hallway;

    double lpf = fmax(1.0,2*M_PI*TS*CUTOFF_FREQ);   // Low pass filter [-]

    int delta_assignment_on_overtake;

    PointID rw_p_rear, rw_p_front, lw_p_rear, lw_p_front;

    int ind, qmax;
    double space_left, space_right, shift_wall, wallang;

public:

    NapoleonPrediction()
    {
    }

    void checkForCollisions(NapoleonModel &M, NapoleonObstacle &O);
    void considerOvertaking(NapoleonAssignment &A, NapoleonObstacle &O);
    void simulateRobotDuringCurrentPredictionStep();
    void start(NapoleonModel &M);
    void initialize(NapoleonModel &M, NapoleonObstacle &O);
    void update(ros::Publisher &ropodmarker_pub, ros::Publisher &wallmarker_pub, NapoleonAssignment &A, NapoleonModel &M, NapoleonObstacle &O, NapoleonVisualization &V, NapoleonPrediction &P);
    bool checkIfFinished(NapoleonAssignment &A);
    void step();

};

#endif
