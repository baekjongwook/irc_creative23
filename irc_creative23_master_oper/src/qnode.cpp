/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/irc_creative23_master_oper/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace irc_creative23_master_oper {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"irc_creative23_master_oper");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    joy_sub = n.subscribe("humo_joy", 100, &QNode::joyCallback, this);
    robot_pub = n.advertise<kubo_msgs::robot2operator>("robot2operator", 100);
    operator_pub = n.advertise<kubo_msgs::operator2robot>("operator2robot", 100);

    start();
    return true;
}

void QNode::run() {
    ros::Rate loop_rate(33);
    while ( ros::ok() ) {

        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::joyCallback(const kubo_msgs::humo::ConstPtr &msg)
{
    joy = *msg;
}

}  // namespace irc_creative23_master_oper
