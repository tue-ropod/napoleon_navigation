#ifndef NAP_TUB_H
#define NAP_TUB_H

using namespace std;

#include <vector>
#include "napoleon_geometry.h"
#include <ed_gui_server/objPosVel.h>
#include <ed_gui_server/objsPosVel.h>

class NapoleonObstacle{

public:

    int no_obs = 0;
    ed_gui_server::objPosVel current_obstacle;
    double obs_theta;
    Point obs_center_global;
    Point current_obs_in_ropod_frame_pos, obs_in_ropod_frame_pos;
    double v_obs_sq = 0;

public:

    NapoleonObstacle(){

    }


};

#endif
