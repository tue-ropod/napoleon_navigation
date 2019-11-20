#ifndef NAP_CONF
#define NAP_CONF

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
using namespace std;

struct NapoleonConfig
{
    // Vehicle size & size of vectors
    double SIZE_SIDE; // How wide vehicle is from center [m]
    double ROPOD_LENGTH; // Ropod length [m]
    double SIZE_FRONT_ROPOD; // How long ropod is from center [m]

    double FEELER_SIZE; // Size of feeler [m] - used to predict where ropod goes suppose it would go straight
    double FEELER_SIZE_STEERING; // Size of feeler when steering [m]
    double ENV_TCTW_SIZE; // Too close too wall area size [m]
    double ENV_TRNS_SIZE; // Transition area size [m]
    double CARROT_LENGTH; // How far ahead point lies where ropod steers towards when too close to a wall [m]

    double D_AX; // Length from rear axle to front steering point [m] (with load)
    double ROPOD_TO_AX; // Length from rear axle to ropod center [m] (with load)
    double SIZE_REAR; // How far vehicle extends behind rear axle (with load)

    // Optimization / performance parameters
    double ENV_TRNS_SIZE_CORNERING; // d_trnsc Transition area size while cornering [m]
    double V_CRUISING; // Max velocity [m/s] while cruising
    double V_INTER_TURNING; // Max velocity [m/s] when taking a turn
    double V_INTER_ACC; // Max velocity [m/s] when driving straight at intersection
    double V_INTER_DEC; // Max velocity [m/s] when driving straight at intersection
    double V_ENTRY; // Max velocity [m/s] when at entry of intersection
    double V_STEERSATURATION; // Velocity during steering saturation [m/s]
    double V_OVERTAKE; // Velocity during overtaking [m/s]

    double DELTA_DOT_LIMIT; // Max steering rate per second [rad/s]
    double A_MAX; // Maximum acceleration magnitude [m/s^2]
    // Obstacle
    double V_OBS_OVERTAKE_MAX; // Max speed an obstacle can have to overtake is [m/s]
    double MIN_DIST_TO_OVERTAKE; // Don't start earlier than x meters to overtake [m]
    // Performance based on position in environment
    double START_STEERING_EARLY_RIGHT; // Start steering earlier by x [m]
    double START_STEERING_EARLY_LEFT; // Start steering earlier by x [m]
    double ROTATED_ENOUGH_THRES; // Stop turning when within x rad of the new corridor [rad]

    double SIZE_FRONT_RAX; // How far vehicle extends in front of rear axle [m]
    double FOLLOW_WALL_DIST_TURNING; // ??

    // Optimization/performance parameters
    double T_MIN_PRED; // Predict for n seconds (unless we want to predict an area) [s]
    double T_PRED_WALL_COLLISION; // Predict for n seconds if ropod collides with walls [s]
    double T_PRED_OBS_COLLISION; // Predict for n seconds if ropod collides with obstacles [s]
    double CUTOFF_FREQ; // Cutoff frequency for low pass filter to simulate steering delay [Hz]
    double OBS_AVOID_MARGIN; // Margin between ropod and obstacles at full speed [m]
    double OBS_AVOID_MARGIN_FRONT; // Margin between ropod front and obstacles at full speed [m]
    double DILATE_ROPOD_ALIGNING; // Dilation from center (so actually this value -size_side if measured from side of vehicle) [m]

    // Fictional hallway width. Ropod will work with lanes of this/2 [m], starting from the wall.
    // No matter what the real hallway size is. This way it will stay close to the right wall, but not too aggressively.
    double TUBE_WIDTH_C; // Default tube width [m]
    double REACHEDTARGETTHRESHOLD; // When x [m] removed from center of last hallway, program finished

    // Performance based on position in environment
    double ENTRY_LENGTH; // Length of entries before intersections [m]
    double SHARP_ANGLE_THRESHOLD; // Angle how much the next hallway must be sharper than perpendicular to be considered sharp [rad]
};

extern struct NapoleonConfig config;
static constexpr double T_MAX_PRED = 20;            // Predict for n seconds max [s]
// Resolutions
static constexpr double F_PLANNER = 10;    // Frequency of planning the motion [Hz]
static constexpr double F_FSTR = 10;   // How many times faster the simulation is than the planning [-]
static constexpr double F_MODEL = F_FSTR*F_PLANNER;   // Frequency of simulation [Hz]
static constexpr double TS = 1/(double)F_MODEL;         // Sample time of model [s]
// Note: in C++ the resolutions are static, however in Matlab they are dynamic
// When C++ implementation desires dynamic resolutions, declare these elsewhere and not as statics
//
static const vector<double> V_SCALE_OPTIONS = {1.0, 0.67, 0.33, 0.0};  // Options to scale velocity with

#endif
