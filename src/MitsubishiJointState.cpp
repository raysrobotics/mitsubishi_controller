/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 2.10.15
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 */

#include <mitsubishi_controller/MitsubishiJointState.h>
#include <boost/asio/read_until.hpp>
#include <sensor_msgs/JointState.h>
#include <boost/algorithm/string.hpp>

MitsubishiJointState::MitsubishiJointState(const std::string &ip, const std::string &port,
                                           boost::asio::io_service &io_service) : stopped(false), socket(io_service),
                                                                                  timer_deadline(io_service),
                                                                                  timer_check_ros(io_service) {
    using boost::asio::ip::tcp;
    pub_joint_state = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    timer_deadline.async_wait(boost::bind(&MitsubishiJointState::check_deadline, this));
    timer_check_ros.expires_from_now(boost::posix_time::milliseconds(TIMER_CHECK_ROS_MS));
    timer_check_ros.async_wait(boost::bind(&MitsubishiJointState::check_ros_ok, this));
    tcp::resolver r(io_service);
    start_connect(r.resolve(tcp::resolver::query(ip, port)));
}

void MitsubishiJointState::stop() {
    stopped = true;
    socket.close();
    timer_deadline.cancel();
    timer_check_ros.cancel();
}

void MitsubishiJointState::start_connect(boost::asio::ip::tcp::resolver::iterator endpoint_iter) {
    using boost::asio::ip::tcp;
    if (endpoint_iter != tcp::resolver::iterator()) {
        ROS_INFO_STREAM("Trying " << endpoint_iter->endpoint());
        socket.async_connect(endpoint_iter->endpoint(),
                             boost::bind(&MitsubishiJointState::handle_connect, this, _1, endpoint_iter));
        timer_deadline.expires_from_now(boost::posix_time::seconds(TIMEOUT_CONNECTION));
    } else {
        stop(); // There are no more endpoints to try. Shut down the client.
    }
}

void MitsubishiJointState::handle_connect(const boost::system::error_code &ec,
                                          boost::asio::ip::tcp::resolver::iterator endpoint_iter) {
    if (stopped) {
        return;
    }

    if (!socket.is_open()) {
        ROS_ERROR_STREAM("Connection time out");
        start_connect(++endpoint_iter);// Try the next available endpoint.
    } else if (ec) { //deadline expired
        ROS_ERROR_STREAM("Connect error: " << ec.message());
        socket.close();// We need to close the socket used in the previous connection attempt before starting a new one
        start_connect(++endpoint_iter);// Try the next available endpoint.
    } else { // Otherwise we have successfully established a connection.
        ROS_INFO_STREAM("Connected to " << endpoint_iter->endpoint());
        start_read();
    }
}

void MitsubishiJointState::check_deadline() {
    if (stopped) {
        return;
    }
    if (timer_deadline.expires_at() <= boost::asio::deadline_timer::traits_type::now()) {
        stop();
    }
    timer_deadline.async_wait(boost::bind(&MitsubishiJointState::check_deadline, this));
}

void MitsubishiJointState::check_ros_ok() {
    if (stopped) {
        return;
    }
    if (!ros::ok()) {
        stop();
        return;
    }

    timer_check_ros.expires_from_now(boost::posix_time::milliseconds(TIMER_CHECK_ROS_MS));
    timer_check_ros.async_wait(boost::bind(&MitsubishiJointState::check_ros_ok, this));
}

void MitsubishiJointState::start_read() {
    timer_deadline.expires_from_now(boost::posix_time::seconds(TIMEOUT_JOINTS_RECEIVED));
    // Start an asynchronous operation to read a newline-delimited message.
    boost::asio::async_read_until(socket, input_buffer, '\r',
                                  boost::bind(&MitsubishiJointState::handle_read, this, _1));
}

void MitsubishiJointState::handle_read(const boost::system::error_code &ec) {
    if (stopped) {
        return;
    }

    if (ec) {
        ROS_ERROR_STREAM("Error on receive joints: " << ec.message());
        stop();
        return;
    }

    auto line = std::string(std::istreambuf_iterator<char>(&input_buffer), std::istreambuf_iterator<char>());
    start_read(); //expect next joint values

    if (line.empty()) {
        return;
    }

    std::vector<std::string> strs;
    boost::split(strs, line, boost::is_any_of(";"));
    std::vector<double> vals(strs.size());
    try {
        std::transform(strs.begin(), strs.end(), vals.begin(), [](const std::string &s) { return std::stod(s); });
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM("Bed message received from controller!" << line);
        stop();
        return;
    }

    sensor_msgs::JointState js;
    js.position = vals;
    js.effort.resize(vals.size(), 0.0);
    js.velocity.resize(vals.size(), 0.0);
    js.name = {"j1", "j2", "j3", "j4", "j5", "j6"};
    js.header.stamp = ros::Time::now();
    pub_joint_state.publish(js);
}
