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
#include "../include/irc_creative23_master_robot/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace irc_creative23_master_robot
{

vector<uchar> cam_buff;
    /*****************************************************************************
    ** Implementation
    *****************************************************************************/

    bool isRecved = false;

    QNode::QNode(int argc, char **argv) : init_argc(argc),
                                          init_argv(argv)
    {
    }

    QNode::~QNode()
    {
        if (ros::isStarted())
        {
            ros::shutdown(); // explicitly needed since we use ros::start();
            ros::waitForShutdown();
        }
        wait();
    }

    bool QNode::init()
    {
        ros::init(init_argc, init_argv, "irc_creative23_master_robot");
        if (!ros::master::check())
        {
            return false;
        }
        ros::start(); // explicitly needed since our nodehandle is going out of scope.
        ros::NodeHandle n;
        // Add your ros communications here.

        image_transport::ImageTransport it(n);
        Cam_sub = it.subscribe("/camera/color/image_raw", 1, &QNode::Cam_Callback, this);

        robot_pub = n.advertise<kubo_msgs::robot2operator>("robot2operator", 100);
        operator_pub = n.advertise<kubo_msgs::operator2robot>("operator2robot", 100);

        start();
        return true;
    }

    void QNode::run()
    {
        ros::Rate loop_rate(33);
        while (ros::ok())
        {

            ros::spinOnce();
            loop_rate.sleep();
        }
        std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
        Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
    }

    void QNode::Cam_Callback(const sensor_msgs::ImageConstPtr &msg_img)
    {
        if (Cam_img == NULL && !isRecved)
        {
            Cam_img = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8)->image);
            if (Cam_img != NULL)
            {
                isRecved = true;
                Original = Cam_img->clone();
                resize(Original, Original, Size(640, 360));
                flip(Original, Original, -1);

                vector<int> param = vector<int>(2);
                param[0] = cv::IMWRITE_JPEG_QUALITY; // incoding type
                param[1] = 60;                       // default(95) 0-100  incoding quality value

                imencode(".jpg", Original, cam_buff, param);

                Q_EMIT Cam_SIGNAL();
            }
        }
    }
} // namespace irc_creative23_master_robot
