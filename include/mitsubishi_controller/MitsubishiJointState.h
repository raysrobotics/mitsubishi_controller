/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 2.10.15
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *    Details: TCP/IP client as a joint state receiver
 *
 *    Server (controller) program, in our case placed from main program into the slot 2:
 *
 *    FileName: JS
 *    //todo copy JS program into comment
 *
 */

#ifndef PROJECT_MITSUBISHIJOINTSTATE_H
#define PROJECT_MITSUBISHIJOINTSTATE_H

#include <ros/ros.h>
#include <boost/asio/io_service.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/ip/tcp.hpp>

class MitsubishiJointState {
public:
    /** \brief Create Joint State TCP/IP client and initialize asynchronous connection procedure */
    MitsubishiJointState(const std::string &ip, const std::string &port, boost::asio::io_service &io_service);

private:
    /** \brief Properly close all streams */
    void stop();

    /** \brief Check deadlines and stop operations if something goes wrong */
    void check_deadline();

    /** \brief Periodically check that ros is operating */
    void check_ros_ok();

    /** \brief Start connection to the server */
    void start_connect(boost::asio::ip::tcp::resolver::iterator endpoint_iter);

    /** \brief Handle connection */
    void handle_connect(const boost::system::error_code &ec, boost::asio::ip::tcp::resolver::iterator endpoint_iter);

    /** \brief Start reading joint states from controller */
    void start_read();

    /** \brief Process msg from controller and publish joint state to the topic */
    void handle_read(const boost::system::error_code &ec);

private:
    ros::NodeHandle nh;
    ros::Publisher pub_joint_state;

    /** \brief Indicate that communication has been stopped */
    bool stopped;

    /** \brief Socket for communication */
    boost::asio::ip::tcp::socket socket;

    /** \brief Input buffer used to store strings delimited by \r */
    boost::asio::streambuf input_buffer;

    /** \brief Asynchronous timers for checking connection timeouts and for ros operations checking */
    boost::asio::deadline_timer timer_deadline, timer_check_ros;


    const long TIMEOUT_CONNECTION = 5L; //stop if connection not established in X seconds
    const long TIMEOUT_JOINTS_RECEIVED = 5L; //stop if joints not received in X seconds
    const long TIMER_CHECK_ROS_MS = 100L; //how often check that ros::ok is true
};


#endif //PROJECT_MITSUBISHIJOINTSTATE_H
