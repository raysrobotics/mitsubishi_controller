/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 12.10.15
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 */

#include <mitsubishi_controller/MitsubishiCommander.h>


MitsubishiCommander::MitsubishiCommander() : socket(io_service), deadline(io_service) {

    deadline.expires_at(boost::posix_time::pos_infin);
    check_deadline();
}

bool MitsubishiCommander::execute_cmd(const std::string &cmd, int n1, int n2) {
    auto res = execute_cmd_res(cmd, n1, n2);
    auto pos = res.find("QoK");
    return pos != std::string::npos;
}

std::string MitsubishiCommander::execute_cmd_res(const std::string &cmd, int n1, int n2) {
    ROS_DEBUG_STREAM("Writing: " << (std::to_string(n1) + ";" + std::to_string(n2) + ";" + cmd));
    write_line(std::to_string(n1) + ";" + std::to_string(n2) + ";" + cmd);
    auto res = read_at_least(3);
    ROS_DEBUG_STREAM("Result: " << res);
    return res;
}

std::string MitsubishiCommander::read_at_least(unsigned int n, boost::posix_time::time_duration timeout) {
    deadline.expires_from_now(timeout);
    boost::system::error_code ec = boost::asio::error::would_block;
    boost::asio::async_read(socket, input_buffer, boost::asio::transfer_at_least(n),
                            boost::lambda::var(ec) = boost::lambda::_1);

    do {
        io_service.run_one();
    } while (ec == boost::asio::error::would_block);

    if (ec) {
        throw boost::system::system_error(ec);
    }
    auto line = std::string(std::istreambuf_iterator<char>(&input_buffer), std::istreambuf_iterator<char>());
    return line;
}

void MitsubishiCommander::check_deadline() {
    if (deadline.expires_at() <= boost::asio::deadline_timer::traits_type::now()) {
        socket.close();
        deadline.expires_at(boost::posix_time::pos_infin);
    }
    deadline.async_wait(boost::bind(&MitsubishiCommander::check_deadline, this));
}

void MitsubishiCommander::write_line(const std::string &line, boost::posix_time::time_duration timeout) {
    std::string data = line + "\r";
    deadline.expires_from_now(timeout);
    boost::system::error_code ec = boost::asio::error::would_block;
    boost::asio::async_write(socket, boost::asio::buffer(data), boost::lambda::var(ec) = boost::lambda::_1);
    do {
        io_service.run_one();
    } while (ec == boost::asio::error::would_block);

    if (ec) {
        throw boost::system::system_error(ec);
    }
}

void MitsubishiCommander::connect(const std::string &ip, const std::string &port,
                                  boost::posix_time::time_duration timeout) {
    using boost::asio::ip::tcp;

    deadline.expires_from_now(timeout);
    auto iter = tcp::resolver(io_service).resolve(tcp::resolver::query(ip, port));
    boost::system::error_code ec;
    for (; iter != tcp::resolver::iterator(); ++iter) {
        socket.close();
        ec = boost::asio::error::would_block;
        socket.async_connect(iter->endpoint(), boost::lambda::var(ec) = boost::lambda::_1);
        do { // Block until the asynchronous operation has completed.
            io_service.run_one();
            if (!ros::ok()) {
                ec = boost::asio::error::timed_out;
            }
        } while (ec == boost::asio::error::would_block);

        if (!ec && socket.is_open()) {
            return;
        }
    }
    throw boost::system::system_error(ec ? ec : boost::asio::error::host_not_found);
}

int MitsubishiCommander::write_trajectory(const trajectory_msgs::JointTrajectory &traj) {
    const auto cmds = std::vector<std::string>({"OPEN=", "CNTLON", "LOAD=ROS", "OVRD=5", "EDATA1 *ROS"});
    if (traj.points.empty()) {
        return 0; //nothing to execute then must be true
    }
    auto indices = get_indices_of_joints(traj);
    if (indices.empty()) {
        return -1;
    }
    for (const auto &point : traj.points) {
        if (!check_point_limits(point, indices)) {
            return -1;
        }
    }

    auto suc(true);
    for (const auto &cmd : cmds) { //open file for writing and put first line to program
        suc &= execute_cmd(cmd);
    }

    auto next_line_id = 2;
    for (int i = 0; i < traj.points.size(); ++i) { //put each position to the variable
        std::stringstream ss;
        ss << "EDATA" << next_line_id++ << " J" << i << "=SETJNT(";
        for (auto index : indices) {
            const auto rounded = (round(traj.points[i].positions[index] * 100000.0) / 100000.0);
            ss << rounded << ",";
        }
        ss << "0,0)";
        suc &= execute_cmd(ss.str());
    }

    suc &= execute_cmd("EDATA" + std::to_string(next_line_id++) + " IF M_SVO<>1 THEN GOTO *ROS");
    suc &= execute_cmd("EDATA" + std::to_string(next_line_id++) + " CNT 1");
    for (int i = 0; i < traj.points.size(); ++i) {
        suc &= execute_cmd("EDATA" + std::to_string(next_line_id++) + " MOV J" + std::to_string(i));
    }
    suc &= execute_cmd("EDATA" + std::to_string(next_line_id++) + " END");
    suc &= execute_cmd("SAVE");
    return suc ? next_line_id - 1 : -1;
}


std::vector<size_t> MitsubishiCommander::get_indices_of_joints(const trajectory_msgs::JointTrajectory &traj) const {
    std::vector<size_t> indices;
    for (const auto &jname : joint_names) {
        auto pos = find(traj.joint_names.begin(), traj.joint_names.end(), jname);
        if (pos == traj.joint_names.end()) {
            ROS_WARN_STREAM("Cannot find joint " << jname << " in the trajectory.");
            indices.clear();
            break;
        }
        auto posi = distance(traj.joint_names.begin(), pos);
        indices.push_back(posi);
    }
    if (indices.size() != joint_names.size()) {
        ROS_WARN_STREAM("Invalid number of joints in the trajectory.");
        indices.clear();
    }
    return indices;
}

bool MitsubishiCommander::check_point_limits(const trajectory_msgs::JointTrajectoryPoint &p,
                                             const std::vector<size_t> &indices) const {
    for (int i = 0; i < indices.size(); ++i) {
        auto value = p.positions[indices[i]]; // value of the i-th joint
        if ((value < limits_min[i]) || (value > limits_max[i])) {
            return false;
        }
    }
    return true;
}

bool MitsubishiCommander::wait_for_programme_completion(const std::string &pname, double period) {
    while (ros::ok()) {
        auto status = execute_cmd_res("STATE");
        if (status.find("QoK") == std::string::npos) { //error of the command
            ROS_WARN_STREAM("Cannot execute state command.");
            return false;
        }
        if (status.find("QoK" + pname) == std::string::npos) { //not running anymore
            break;
        }
        ros::Duration(period).sleep();
    }
    return true;
}

bool MitsubishiCommander::start_joint_streamer() {
    auto suc(true);
    suc &= execute_cmd("OPEN=");
    suc &= execute_cmd("CNTLON");
    execute_cmd("SLOTINIT");
    suc &= execute_cmd("RUNMAIN;1");
    suc &= wait_for_programme_completion("MAIN");
    return suc;
}