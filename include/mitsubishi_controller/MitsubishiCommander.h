/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 12.10.15
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *    Details: Execute command in the controller.
 */

#ifndef PROJECT_MITSUBISHICOMMANDER_H
#define PROJECT_MITSUBISHICOMMANDER_H

#include <boost/asio.hpp>
#include <boost/lambda/lambda.hpp>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

class MitsubishiCommander {

public:
    /** \brief Initialize io service for socket and deadline */
    MitsubishiCommander();

    /** \brief Execute command in the controller and compare wether it returns QoK string.
     *  \return false if failed to send/complete command. */
    bool execute_cmd(const std::string &cmd, int n1 = 1, int n2 = 1);

    /** \brief Execute command and get result in form of string.
     *  \details For non-common commands only, otherwise use \e execute_cmd with integrated result checker */
    std::string execute_cmd_res(const std::string &cmd, int n1 = 1, int n2 = 1);

    /** \brief Connecto to the controller ip/port. Throw exception if timeout or ros cancelled */
    void connect(const std::string &ip, const std::string &port,
                 boost::posix_time::time_duration timeout = boost::posix_time::seconds(5L));

    /** \brief Write trajectory to the controller as the executable program.
     *  \return number of written lines or -1 on error */
    int write_trajectory(const trajectory_msgs::JointTrajectory &traj);

    /** \brief Wait until the program \e pname does not finish execution.
     *  \return true if successfully completed, false on error */
    bool wait_for_programme_completion(const std::string& pname, double period = 0.1);

    /** \brief Start joint streamer programme on the controller side */
    bool start_joint_streamer();

private:
    /** \brief Read at least \e n bytes from socket. Throw system error if failed */
    std::string read_at_least(unsigned int n = 3,
                              boost::posix_time::time_duration timeout = boost::posix_time::seconds(5L));

    /** \brief Write line to the stream (\r is appended in fnct.) throw error on fail */
    void write_line(const std::string &line,
                    boost::posix_time::time_duration timeout = boost::posix_time::seconds(5L));

    /** \brief Check deadline timer for read/write operations */
    void check_deadline();

    /** \brief Extract indices of the joints as they are ordered in the trajectory.
     *  \details if joint "j2" is located on the fifth position in trajectory then result[1] = 5. */
    std::vector<size_t> get_indices_of_joints(const trajectory_msgs::JointTrajectory &traj) const;

    /** \brief Check that point \p is in the controller joint limits given the order of joints specified in \e indices*/
    bool check_point_limits(const trajectory_msgs::JointTrajectoryPoint &p, const std::vector<size_t> &indices) const;

public:
    /** \brief Joint limits in radians */
    std::vector<double> limits_min, limits_max;

    /** \brief Joint names read from the parameter server or j1/j2/.../j6 by default */
    std::vector<std::string> joint_names;

private:
    boost::asio::io_service io_service;
    boost::asio::ip::tcp::socket socket;
    boost::asio::deadline_timer deadline;
    boost::asio::streambuf input_buffer;


};


#endif //PROJECT_MITSUBISHICOMMANDER_H
