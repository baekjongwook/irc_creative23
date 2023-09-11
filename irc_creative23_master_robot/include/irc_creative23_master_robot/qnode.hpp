/**
 * @file /include/irc_creative23_master_robot/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef irc_creative23_master_robot_QNODE_HPP_
#define irc_creative23_master_robot_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include <vector>

#include <kubo_msgs/humo.h>
#include <kubo_msgs/operator2robot.h>
#include <kubo_msgs/robot2operator.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace irc_creative23_master_robot {
using namespace cv;
using namespace std;
using namespace Qt;

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
    Mat *Cam_img;
    Mat Original;
        void run();


Q_SIGNALS:
    void rosShutdown();
    void Cam_SIGNAL(void);

private:
	int init_argc;
	char** init_argv;

    image_transport::Subscriber Cam_sub;
    void Cam_Callback(const sensor_msgs::ImageConstPtr& msg_img);

public:
  ros::Publisher robot_pub;
  ros::Publisher operator_pub;

  kubo_msgs::robot2operator robotMsg;
  kubo_msgs::operator2robot operatorMsg;

};

}  // namespace irc_creative23_master_robot

#endif /* irc_creative23_master_robot_QNODE_HPP_ */
