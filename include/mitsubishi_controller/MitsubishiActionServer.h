/**
 * Copyright (c) CTU in Prague  - All Rights Reserved
 * Created on: 14.10.15
 *     Author: Vladimir Petrik <petrivl3@fel.cvut.cz>
 *    Details: Actionlib interface to moveit
 */

#ifndef PROJECT_MITSUBISHIACTIONSERVER_H
#define PROJECT_MITSUBISHIACTIONSERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mitsubishi_controller/MitsubishiCommander.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

class MitsubishiActionServer {
public:
    MitsubishiActionServer();

    bool start();

    void cb_execute(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

private:
    ros::NodeHandle node;
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    MitsubishiCommander mc;

};


#endif //PROJECT_MITSUBISHIACTIONSERVER_H
