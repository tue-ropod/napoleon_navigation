#include "napoleon_visualization.h"

void NapoleonVisualization::visualizeRopodMarkers()
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

void NapoleonVisualization::showWallPoints(Point local_wallpoint_front, Point local_wallpoint_rear,  ros::Publisher &pub) {
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

void NapoleonVisualization::initializeVisualizationMarkers()
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

void NapoleonVisualization::publish(ros::Publisher &ropodmarker_pub){
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
    vis_p.x = A.glob_wallpoint_rear.x; vis_p.y = A.glob_wallpoint_rear.y; vis_points.points.push_back(vis_p);
    vis_p.x = A.glob_wallpoint_front.x; vis_p.y = A.glob_wallpoint_front.y; vis_points.points.push_back(vis_p);
    ropodmarker_pub.publish(vis_points);
    vis_points.points.clear();
    // End publish ropod points
    
}

void NapoleonVisualization::visualizePlan(ros::Publisher &mapmarker_pub, NapoleonAssignment &A, NapoleonStatemachine &S){
    // Plan visualization
    vis_plan.points.clear();
    vis_plan.header.stamp = ros::Time::now();
    vis_plan.id = 10*S.i+S.j;
    geometry_msgs::Point p;
    int points_size = A.pointlist.size();

    p.x = A.pointlist[points_size-4].x;
    p.y = A.pointlist[points_size-4].y;
    vis_plan.points.push_back(p);
    p.x = A.pointlist[points_size-3].x;
    p.y = A.pointlist[points_size-3].y;
    vis_plan.points.push_back(p);
    p.x = A.pointlist[points_size-2].x;
    p.y = A.pointlist[points_size-2].y;
    vis_plan.points.push_back(p);
    p.x = A.pointlist[points_size-1].x;
    p.y = A.pointlist[points_size-1].y;
    vis_plan.points.push_back(p);
    p.x = A.pointlist[points_size-4].x;
    p.y = A.pointlist[points_size-4].y;
    vis_plan.points.push_back(p);
    mapmarker_pub.publish(vis_plan);
}
