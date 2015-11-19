/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 2.10.15
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 */

#include <ros/ros.h>
#include <mitsubishi_controller/MitsubishiJointState.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "mitsubishi_joint_state");
    ros::NodeHandle nh;

    const auto to_rad = M_PI / 180.0;
    std::vector<double> limits_min = {-170.0 * to_rad, -92.0 * to_rad, -129.0 * to_rad, -160.0 * to_rad,
                                      -120.0 * to_rad, -360.0 * to_rad};

    std::vector<double> limits_max = {170.0 * to_rad, 135.0 * to_rad, 166.0 * to_rad, 160.0 * to_rad, 120.0 * to_rad,
                                      360.0 * to_rad};

    for (auto l : limits_min) {
        std::cout << l << ", ";
    }
    std::cout << "" << std::endl;

    for (auto l : limits_max) {
        std::cout << l << ", ";
    }
    std::cout << "" << std::endl;

    std::string ip, port;
    nh.param<std::string>("ip", ip, "192.168.0.20");
    nh.param<std::string>("port", port, "10004");

    try {
        boost::asio::io_service io_service;
        MitsubishiJointState js(ip, port, io_service);
        io_service.run();
    } catch (std::exception &e) {
        ROS_ERROR_STREAM("Exception: " << e.what());
    }

    return 0;
}
