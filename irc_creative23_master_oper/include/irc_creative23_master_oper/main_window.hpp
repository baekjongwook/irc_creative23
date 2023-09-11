/**
 * @file /include/irc_creative23_master_oper/main_window.hpp
 *
 * @brief Qt based gui for irc_creative23_master_oper.
 *
 * @date November 2010
 **/
#ifndef irc_creative23_master_oper_MAIN_WINDOW_H
#define irc_creative23_master_oper_MAIN_WINDOW_H

#define PI 3.14159265

#define DEFALT_WORK             0        // 0: OPERATOR, 1: ROBOT
#define LINEAR_VELOCITY_LIMIT   100      // cm/s
#define ANGULAR_VELOCITY_LIMIT  1        // rad/s
#define RPM_LIMIT_H             40        // rpm
#define RPM_LIMIT_L             0       // rpm
#define ROBOT_WIDTH             50       // cm
#define WHEEL_RADIUS            11.8     // cm
#define PRECISION               1000     // 10^-3 point precision
#define PAN_ANGLE_LIMIT         60       // degree
#define TILT_ANGLE_LIMIT_H      30       // degree
#define TILT_ANGLE_LIMIT_L      -10      // degree

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#include <QtNetwork/QUdpSocket>
#include <QtNetwork/QHostAddress>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace irc_creative23_master_oper {
using namespace cv;
using namespace Qt;
using namespace std;

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    void udp_write(QString text);

public Q_SLOTS:
    void master();

    void udp_read();
    void udp_cam_read();
    void udp_thermal_read();
    Mat Decoding_Datagram(QByteArray inputDatagram);
    void combineDatagram(QByteArray inputDatagram);
    void showVideo();

private:
    Ui::MainWindowDesign ui;
    QNode qnode;

    QTimer *m_Timer;

    QUdpSocket *socket;
    QUdpSocket *cameraSocket;
    QUdpSocket *thermalSocket;

    int updateFlag_UCAM=0;

    QString text_Data=0;

    Mat udp_image;
    Mat udp_thermal_image;

    QByteArray buffer;
    QByteArray image_buffer;

    quint8 image_cnt_past=0;
    quint8 image_cnt_now=0;

    QTimer *_5ms_Timer, *_1s_Timer;

    double map(double value, double fromLow, double fromHigh, double toLow, double toHigh);
};

}  // namespace irc_creative23_master_oper

#endif // irc_creative23_master_oper_MAIN_WINDOW_H
