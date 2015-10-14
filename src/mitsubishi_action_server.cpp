/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 13.10.15
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 */

#include <ros/ros.h>
#include <mitsubishi_controller/MitsubishiActionServer.h>
#include <mitsubishi_controller/MitsubishiCommander.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "tcptest");
    ros::NodeHandle node("~");

    MitsubishiActionServer as;
    as.start();
    ros::spin();

//    try {
//        MitsubishiCommander mc;
//        mc.connect("192.168.0.20", "10002");
//
//        if (!mc.start_joint_streamer()) {
//            ROS_ERROR_STREAM("Cannot start joint streamer programme on controller.");
//            return -1;
//        }
//
//
//        trajectory_msgs::JointTrajectory t;
//        t.joint_names.push_back("j1");
//        t.joint_names.push_back("j2");
//        t.joint_names.push_back("j3");
//        t.joint_names.push_back("j4");
//        t.joint_names.push_back("j5");
//        t.joint_names.push_back("j6");
//        t.points.resize(3);
//        t.points[0].positions = {0.00769867, 0.0896933, 1.58429, 0.083742, 0.74036, -0.0719543};
//        t.points[1].positions = {0.00769867, 0.0896933, 1.58429, 0.083742, 0.64036, -0.2719543};
//        t.points[2].positions = {0.00769867, 0.0896933, 1.58429, 0.083742, 0.74036, -0.0719543};
//
//        auto num_lines = mc.write_trajectory(t);
//        if (num_lines < 0) {
//            ROS_WARN_STREAM("Cannot write trajectory.");
//            return -1;
//        }
//
//        mc.execute_cmd("SRVON");
//        mc.execute_cmd("SLOTINIT");
//        mc.execute_cmd("RUNROS;1");
//        while (ros::ok()) {
//            auto status = mc.execute_cmd_res("STATE");
//            if (status.find("QoK") == std::string::npos) { //error of the command
//                ROS_WARN_STREAM("Cannot execute state command.");
//                break;
//            }
//            if (status.find("QoKROS") == std::string::npos) { //not running anymore
//                break;
//            }
//            ros::Duration(0.1).sleep();
//        }
//        mc.execute_cmd("SRVOFF");
//    } catch (std::exception &e) {
//        ROS_ERROR_STREAM("Exception: " << e.what());
//    }

    return 0;
}
