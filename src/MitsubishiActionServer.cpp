/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 14.10.15
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 */

#include <mitsubishi_controller/MitsubishiActionServer.h>


MitsubishiActionServer::MitsubishiActionServer() :
        node("~"),
        as_(node, "follow_joint_trajectory", boost::bind(&MitsubishiActionServer::cb_execute, this, _1), false) {
}

bool MitsubishiActionServer::start() {

    std::string ip, port;
    node.param<std::string>("ip", ip, "192.168.0.20");
    node.param<std::string>("port", port, "10002");

    if (!node.getParam("joint_names", mc.joint_names)) {
        ROS_INFO_STREAM("Joint Names are not specified, using defaults");
        mc.joint_names = {"j1", "j2", "j3", "j4", "j5", "j6"};
    }

    if (!node.getParam("limits_min", mc.limits_min)) {
        const auto to_rad = M_PI / 180.0;
        mc.limits_min = {-170.0 * to_rad, -92.0 * to_rad, -129.0 * to_rad, -160.0 * to_rad, -120.0 * to_rad,
                         -360.0 * to_rad};
    }

    if (!node.getParam("limits_max", mc.limits_max)) {
        const auto to_rad = M_PI / 180.0;
        mc.limits_max = {170.0 * to_rad, 135.0 * to_rad, 166.0 * to_rad, 160.0 * to_rad, 120.0 * to_rad,
                         360.0 * to_rad};
    }

    try {
        mc.connect(ip, port);
        if (!mc.start_joint_streamer()) {
            ROS_ERROR_STREAM("Cannot start joint trajectory streamer");
            return false;
        }
    } catch (std::exception &e) {
        ROS_ERROR_STREAM("Cannot connect: " << e.what());
        return false;
    }

    as_.start();
    return true;
}

void MitsubishiActionServer::cb_execute(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal) {

    try {
        auto num_lines = mc.write_trajectory(goal->trajectory);
        if (num_lines < 0) {
            ROS_WARN_STREAM("Cannot write trajectory.");
            as_.setAborted();
            return;
        }

        auto suc(true);
        suc &= mc.execute_cmd("SRVON");
        mc.execute_cmd("SLOTINIT");
        suc &= mc.execute_cmd("RUNROS;1");
        if (!suc) {
            as_.setAborted();
            return;
        }
        suc &= mc.wait_for_programme_completion("ROS");
        suc &= mc.execute_cmd("SRVOFF");

        if (!suc) {
            as_.setAborted();
            return;
        }
    } catch (std::exception &e) {
        ROS_ERROR_STREAM("Exception: " << e.what());
        as_.setAborted();
    }
    as_.setSucceeded();
}
