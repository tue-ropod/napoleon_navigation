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
