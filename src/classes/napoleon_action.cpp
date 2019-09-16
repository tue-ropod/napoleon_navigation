#include "napoleon_action.h"

void NapoleonAction::update(ros::Publisher &vel_pub, NapoleonModel &M, NapoleonPrediction &P)
{
    if (M.control_v > 0) {
        M.v_ax = cos(P.pred_phi_des[1])*M.control_v;
        M.theta_dot = M.control_v/D_AX*sin(P.pred_phi_des[1]);
        cmd_vel.linear.x = M.v_ax;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = M.theta_dot;
        vel_pub.publish(cmd_vel);
    } else {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_pub.publish(cmd_vel);
    }
}

void NapoleonAction::stop(ros::Publisher &vel_pub)
{
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub.publish(cmd_vel);
}
