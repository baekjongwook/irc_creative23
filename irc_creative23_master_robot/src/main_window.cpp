/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/irc_creative23_master_robot/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace irc_creative23_master_robot
{

extern vector<uchar> cam_buff;
  extern bool isRecved;
  using namespace Qt;

  string STR_USB_CAM = "USB CAM";

  QHostAddress ROBOT_IP = QHostAddress("192.168.1.7");    // orange
  QHostAddress OPERATOR_IP = QHostAddress("192.168.1.6"); // black

  uint16_t ROBOT_PORT = 9999;
  uint16_t other_PORT = 8888;

  /*****************************************************************************
  ** Implementation [MainWindow]
  *****************************************************************************/

  MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
      : QMainWindow(parent), qnode(argc, argv)
  {
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    setWindowIcon(QIcon(":/images/icon.png"));

    qnode.init();

    socket = new QUdpSocket(this);
    m_pUdpSocket = new QUdpSocket(this);

    if (socket->bind(ROBOT_IP, ROBOT_PORT))
    {
      connect(socket, SIGNAL(readyRead()), this, SLOT(udp_read()));
    }

    m_pUdpSocket->bind(ROBOT_IP, other_PORT, QUdpSocket::ShareAddress);

    QObject::connect(&qnode, SIGNAL(Cam_SIGNAL()), this, SLOT(Cam_SLOT()));

    QObject::connect(this, SIGNAL(view_SIGNAL()), this, SLOT(Show_usb_cam()));

    m_Timer = new QTimer(this);
    connect(m_Timer, SIGNAL(timeout()), this, SLOT(master()));
    m_Timer->start(100);

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  }

  MainWindow::~MainWindow()
  {

    delete m_Timer;
    delete _5ms_Timer;
    delete _1s_Timer;
  }

  /*****************************************************************************
  ** Functions
  *****************************************************************************/

  void MainWindow::master()
  {
    //-------------------------------------------------------
    // UDP READ

    //-------------------------------------------------------
    // UDP SEND
    //    QByteArray packet;

    //    packet.push_back(qnode.operatorMsg.left_rpm - RPM_LIMIT);
    //    packet.push_back(qnode.operatorMsg.right_rpm - RPM_LIMIT);
    //    packet.push_back(qnode.operatorMsg.pan_angle - PAN_ANGLE_LIMIT);
    //    packet.push_back(qnode.operatorMsg.tilt_angle - TILT_ANGLE_LIMIT_H);
    //    packet.push_back(qnode.operatorMsg.isFire);
    //    packet.push_back(qnode.operatorMsg.isLightOn);

    //    socket->writeDatagram(packet, ROBOT_IP, ROBOT_PORT);

    //----------------------------------------------------------

    // MSG PUBLISH
    qnode.robot_pub.publish(qnode.robotMsg);
    qnode.operator_pub.publish(qnode.operatorMsg);
  }

  void MainWindow::Cam_SLOT()
  {
    QImage raw_image((const unsigned char *)(qnode.Original.data), qnode.Original.cols, qnode.Original.rows, QImage::Format_RGB888);
    ui.label_camTest->setPixmap(QPixmap::fromImage(raw_image.rgbSwapped()));

    delete qnode.Cam_img;
    if (qnode.Cam_img != NULL)
      qnode.Cam_img = NULL;
    isRecved = false;

    Q_EMIT view_SIGNAL();
  }

  void MainWindow::udp_write(QString text)
  {

    QByteArray packet;
    packet.append(text);
    qDebug() << "Message from: udp_write";
    socket->writeDatagram(packet, OPERATOR_IP, ROBOT_PORT);
    usleep(1);
  }

  void MainWindow::udp_read()
  {
    cout << "!!UDP READ OK!!:" << endl;

    QByteArray buffer;
    buffer.resize(socket->pendingDatagramSize());

    socket->readDatagram(buffer.data(), buffer.size(), &OPERATOR_IP, &ROBOT_PORT);

    const std::size_t count = buffer.size();
    unsigned char *RxData = new unsigned char[count];
    std::memcpy(RxData, buffer.constData(), count);

    double left_rpm_temp = RxData[0] - 40;
    double right_rpm_temp = RxData[1] - 40;
    double pan_angle_temp = RxData[2] - 60;
    double tilt_angle_temp = RxData[3] - 30;
    bool isFire_temp = RxData[4];
    bool isLightOn_temp = RxData[5];

    delete[] RxData;

    if (/*abs(left_rpm_temp) <= 40*/ true)
    {
      qnode.operatorMsg.left_rpm = left_rpm_temp;
      qnode.operatorMsg.right_rpm = right_rpm_temp;
      qnode.operatorMsg.pan_angle = pan_angle_temp;
      qnode.operatorMsg.tilt_angle = tilt_angle_temp;
      qnode.operatorMsg.isFire = isFire_temp;
      qnode.operatorMsg.isLightOn = isLightOn_temp;
    }

    cout << "left_rpm: " << qnode.operatorMsg.left_rpm << endl;
    cout << "right_rpm: " << qnode.operatorMsg.right_rpm << endl;
    cout << "pan_angle: " << qnode.operatorMsg.pan_angle << endl;
    cout << "tilt_angle: " << qnode.operatorMsg.tilt_angle << endl;
    cout << "isFire: " << qnode.operatorMsg.isFire << endl;
    cout << "isLight: " << qnode.operatorMsg.isLightOn << endl;
  }

  void MainWindow::Show_usb_cam()
  {
    // qDebug() << "show_usb_cam clear";
    QByteArray usb_cam_image_array;
//    for (int i = 0; i < qnode.Original.rows; i++) // 240 -> 480?
//    {
//      for (int j = 0; j < qnode.Original.cols; j++) // 320 -> 640?
//      {
//        usb_cam_image_array.push_back((unsigned char)(qnode.Original.at<Vec3b>(i, j)[0]));
//        usb_cam_image_array.push_back((unsigned char)(qnode.Original.at<Vec3b>(i, j)[1]));
//        usb_cam_image_array.push_back((unsigned char)(qnode.Original.at<Vec3b>(i, j)[2]));
//      }
//    }

    // qDebug() << (unsigned char)usb_cam_image_array.at(3000);

    /*
     * -----------solved-----------
     * 문제점 : 캠 이미지를 받고 1차원 배열 분해 작업을 해야하는데 캠 이미지를 받지 못한 상태에서 배열 분해 작업을 하려함. Original에는 아무것도 들어 있지 않은 상태에서 at()으로 작업을 하기 때문에 core dumped 발생
     *
     */

    // qDebug() << qnode.Original.rows;
    sendVideo(usb_cam_image_array, other_PORT);

//    usb_cam_image_array.clear();
  }

  void MainWindow::sendVideo(QByteArray img, uint16_t port)
  {
    // copy and convert vector<uchar> to QByteArray
    QByteArray array;
    for (auto val : cam_buff)
    {
      array.push_back(val);
    }
    int sendok;
    sendok = m_pUdpSocket->writeDatagram(array.data(), array.size(), OPERATOR_IP, other_PORT);
    while (sendok == -1)
    {
      sendok = m_pUdpSocket->writeDatagram(array.data(), array.size(), OPERATOR_IP, other_PORT);
    }
    usleep(1);
  }

} // namespace irc_creative23_master_robot
