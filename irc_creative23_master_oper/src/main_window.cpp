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
#include "../include/irc_creative23_master_oper/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace irc_creative23_master_oper
{

string STR_USB_CAM = "USB CAM";

QHostAddress ROBOT_IP = QHostAddress("192.168.1.7");    // orange
QHostAddress OPERATOR_IP = QHostAddress("192.168.1.6"); // black

uint16_t ROBOT_PORT = 9999;
uint16_t CAMERA_PORT = 8888;
uint16_t THERMAL_PORT = 8889;

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
    cameraSocket = new QUdpSocket(this);
    thermalSocket = new QUdpSocket(this);

    _5ms_Timer = new QTimer(this);
    connect(_5ms_Timer, SIGNAL(timeout()), this, SLOT(showVideo())); // 1
    _5ms_Timer->start(5);

    if (socket->bind(OPERATOR_IP, ROBOT_PORT, QUdpSocket::ShareAddress))
    {
        qDebug() << "text message socket bind success" << endl;
        connect(socket, SIGNAL(readyRead()), this, SLOT(udp_read()));
    }

    if (cameraSocket->bind(OPERATOR_IP, CAMERA_PORT, QUdpSocket::ShareAddress))
    {
        qDebug() << "cam image socket bind success" << endl;
        connect(cameraSocket, SIGNAL(readyRead()), this, SLOT(udp_cam_read()));
    }

    if (thermalSocket->bind(OPERATOR_IP, THERMAL_PORT, QUdpSocket::ShareAddress))
    {
        qDebug() << "thermal image socket bind success" << endl;
        connect(thermalSocket, SIGNAL(readyRead()), this, SLOT(udp_thermal_read()));
    }

    udp_image = cv::Mat::ones(240, 320, CV_8UC3);
    udp_thermal_image = cv::Mat::ones(240, 320, CV_8UC3);

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
    //---------------------------------------------
    // ROBOT
    // 1. set rpm L and H
    const double rpm_L = RPM_LIMIT_L;
    const double rpm_H = RPM_LIMIT_H;

    // 2. calc velocity L and H
    const double linear_L = 0.033 * PI * WHEEL_RADIUS * rpm_L; // cm/sec
    const double linear_H = 0.033 * PI * WHEEL_RADIUS * rpm_H; // cm/sec
    const double angular_L = linear_L / (ROBOT_WIDTH / 2);     // rad/sec
    const double angular_H = linear_H / (ROBOT_WIDTH / 2);     // rad/sec

    // 3. mapping joystick value to velocity

    double v_value;
    double w_value;
    if (ui.checkBox_test->isChecked())
    {
        cv::resize(udp_image, udp_image, cv::Size(640, 360));
        v_value = ui.doubleSpinBox_v->value();
        w_value = ui.doubleSpinBox_w->value();
    }
    else
    {
        v_value = qnode.joy.leftStickY;
        w_value = qnode.joy.leftStickX;
    }

    double linear = v_value ? map(fabs(v_value), 0, 1, linear_L, linear_H) : 0;
    double angular = w_value ? map(fabs(w_value), 0, 1, angular_L, angular_H) : 0;

    if (v_value < 0)
        linear *= -1;
    if (w_value < 0)
        angular *= -1;

    // 4. calc RPM
    int left_rpm = static_cast<int>((linear - angular * ROBOT_WIDTH / 2) / (2 * PI * WHEEL_RADIUS) * 60);
    int right_rpm = static_cast<int>((linear + angular * ROBOT_WIDTH / 2) / (2 * PI * WHEEL_RADIUS) * 60);

    // 5. exp
    if (abs(left_rpm) > RPM_LIMIT_H)
    {
        ui.label_L->setStyleSheet("QLabel { background-color : red; color : blue; }");
        left_rpm = left_rpm > 0 ? RPM_LIMIT_H : -RPM_LIMIT_H;
    }
    else
    {
        ui.label_L->setStyleSheet("QLabel { background-color : transparent; color : black; }");
    }
    if (abs(right_rpm) > RPM_LIMIT_H)
    {
        ui.label_R->setStyleSheet("QLabel { background-color : red; color : blue; }");
        right_rpm = right_rpm > 0 ? RPM_LIMIT_H : -RPM_LIMIT_H;
    }
    else
    {
        ui.label_R->setStyleSheet("QLabel { background-color : transparent; color : black; }");
    }

    // 6. ui update
    ui.verticalSlider->setMaximum(linear_H);
    ui.verticalSlider->setMinimum(-linear_H);
    ui.horizontalSlider->setMaximum(angular_H * PRECISION);
    ui.horizontalSlider->setMinimum(-angular_H * PRECISION);
    ui.verticalSlider_L->setMaximum(RPM_LIMIT_H * PRECISION);
    ui.verticalSlider_L->setMinimum(-RPM_LIMIT_H * PRECISION);
    ui.verticalSlider_R->setMaximum(RPM_LIMIT_H * PRECISION);
    ui.verticalSlider_R->setMinimum(-RPM_LIMIT_H * PRECISION);

    ui.verticalSlider->setValue(linear);
    ui.horizontalSlider->setValue(-1 * angular * PRECISION);

    ui.label_v->setNum(linear);
    ui.label_w->setNum(angular);

    ui.verticalSlider_L->setValue(left_rpm * PRECISION);
    ui.verticalSlider_R->setValue(right_rpm * PRECISION);

    ui.label_L->setNum(left_rpm);
    ui.label_R->setNum(right_rpm);

    // 6. adv ros msgs
    qnode.operatorMsg.left_rpm = left_rpm;
    qnode.operatorMsg.right_rpm = right_rpm;

    //------------------------------------------------------
    // TURRET

    const double pan_weight = 6.0;
    const double tilt_weight = 2.0;

    static double pan_angle = 0;
    static double tilt_angle = 0;

    double pan_joy = qnode.joy.rightStickY * pan_weight * -1;
    double tilt_joy = qnode.joy.rightStickX * tilt_weight;

    if (!qnode.joy.buttonRightJoy)
    {
        pan_joy *= 0.5;
        tilt_joy *= 0.5;
    }

    pan_angle += pan_joy;
    tilt_angle += tilt_joy;

    if (abs(pan_angle) > PAN_ANGLE_LIMIT)
    {
        pan_angle = pan_angle > 0 ? PAN_ANGLE_LIMIT : -PAN_ANGLE_LIMIT;
    }

    if (tilt_angle > TILT_ANGLE_LIMIT_H)
    {
        tilt_angle = TILT_ANGLE_LIMIT_H;
    }
    else if (tilt_angle < TILT_ANGLE_LIMIT_L)
    {
        tilt_angle = TILT_ANGLE_LIMIT_L;
    }

    if (qnode.joy.buttonSq)
    {
        pan_angle = 0;
        tilt_angle = 0;
    }

    qnode.operatorMsg.pan_angle = pan_angle;
    qnode.operatorMsg.tilt_angle = tilt_angle;

    ui.dial_pan->setValue(pan_angle);
    ui.verticalSlider_tilt->setValue(tilt_angle);

    ui.label_PAN->setNum(pan_angle);
    ui.label_TILT->setNum(tilt_angle);

    //------------------------------------------------------
    // PERIPHERAL

    static bool isLight = false;
    bool isFire = false;

    static bool previous_buttonTr = false;
    if (previous_buttonTr == 0 && qnode.joy.buttonTr == 1) // catching falling edge
    {
        isLight = !isLight;
    }
    ui.radioButton_LIGHT->setChecked(isLight);

    if(isLight)
    {
        ui.label_LIGHT_STATE->setText("ON");
    }
    else
    {
        ui.label_LIGHT_STATE->setText("OFF");
    }

    previous_buttonTr = qnode.joy.buttonTr;

    if(qnode.robotMsg.isLockon || qnode.joy.buttonO)
    {
        ui.radioButton_FIRE->setEnabled(true);
        ui.label_FIRE_STATE->setText("READY TO FIRE");
        if (qnode.joy.l2 && qnode.joy.r2)
        {
            isFire = true;
        }
    }
    else
    {
        ui.radioButton_FIRE->setEnabled(false);
        ui.label_FIRE_STATE->setText("SAFTY");
    }

    ui.radioButton_FIRE->setChecked(isFire);

    qnode.operatorMsg.isFire = isFire;
    qnode.operatorMsg.isLightOn = isLight;

    //-------------------------------------------------------
    // UDP SEND
    QByteArray packet;

    packet.push_back(qnode.operatorMsg.left_rpm + RPM_LIMIT_H);
    packet.push_back(qnode.operatorMsg.right_rpm + RPM_LIMIT_H);
    packet.push_back(qnode.operatorMsg.pan_angle + PAN_ANGLE_LIMIT);
    packet.push_back(qnode.operatorMsg.tilt_angle + TILT_ANGLE_LIMIT_H);
    packet.push_back(qnode.operatorMsg.isFire);
    packet.push_back(qnode.operatorMsg.isLightOn);

    socket->writeDatagram(packet, ROBOT_IP, ROBOT_PORT);

    //----------------------------------------------------------
    // MSG PUBLISH
    qnode.robot_pub.publish(qnode.robotMsg);
    qnode.operator_pub.publish(qnode.operatorMsg);
}

void MainWindow::udp_read()
{
//    cout << "UDP READ" << endl;

    QByteArray buffer;
    buffer.resize(socket->pendingDatagramSize());

    socket->readDatagram(buffer.data(), buffer.size(), &ROBOT_IP, &ROBOT_PORT);

    const std::size_t count = buffer.size();
    unsigned char *RxData = new unsigned char[count];
    std::memcpy(RxData, buffer.constData(), count);

    qnode.robotMsg.target_x = RxData[1]<<8;
    qnode.robotMsg.target_x += RxData[0];
    qnode.robotMsg.target_y = RxData[3]<<8;
    qnode.robotMsg.target_y += RxData[2];
    qnode.robotMsg.distance = RxData[5]<<8;
    qnode.robotMsg.distance += RxData[4];
    qnode.robotMsg.isLockon = RxData[6];

//    cout << "isLockon: " << qnode.robotMsg.isLockon << endl;

//    cout << "target_x: " << qnode.robotMsg.target_x << endl;
//    cout << "target_y: " << qnode.robotMsg.target_y << endl;

    delete[] RxData;
}

void MainWindow::udp_cam_read()
{cout << "isLockon: " << qnode.robotMsg.isLockon << endl;

    cout << "target_x: " << qnode.robotMsg.target_x << endl;
    cout << "target_y: " << qnode.robotMsg.target_y << endl;
    // qDebug() << "UDP CAM READ";
    QByteArray cam_buffer;
    cam_buffer.resize(cameraSocket->pendingDatagramSize());

    cameraSocket->readDatagram(cam_buffer.data(), cam_buffer.size(), &ROBOT_IP, &CAMERA_PORT);

    udp_image = Decoding_Datagram(cam_buffer);

    cam_buffer.clear();
}

void MainWindow::udp_thermal_read()
{
    QByteArray cam_buffer;
    cam_buffer.resize(thermalSocket->pendingDatagramSize());

    thermalSocket->readDatagram(cam_buffer.data(), cam_buffer.size(), &ROBOT_IP, &THERMAL_PORT);

    udp_thermal_image = Decoding_Datagram(cam_buffer);

    cam_buffer.clear();
}

Mat MainWindow::Decoding_Datagram(QByteArray inputDatagram)
{
    std::vector<uchar> cam_decoding(inputDatagram.begin(), inputDatagram.end());
    Mat image = imdecode(Mat(cam_decoding), IMREAD_COLOR);

    return image;
}

void MainWindow::combineDatagram(QByteArray inputDatagram)
{
    image_cnt_now = (quint8)inputDatagram.at(0);
    //    qDebug() << "Image count now: " << (quint8)image_cnt_now;

    if ((image_cnt_now == (image_cnt_past + 1)) && (image_cnt_now < 76))
    {
        //        qDebug() << "Image count now: " << (quint8)image_cnt_now;
        inputDatagram.remove(0, 1); // remove packet count array[0]
        // qDebug() << "size2: "<<inputDatagram.size();
        image_buffer.append(inputDatagram);

        image_cnt_past = image_cnt_now;
    }

    if (image_cnt_past == 75)
    {
        qDebug() << image_buffer.size();

        int cnt = 0;
        int cnt_for = 1;

        for (int i = 0; i < 360; i++)
        {
            for (int j = 0; j < 640; j++)
            {
                udp_image.at<Vec3b>(i, j)[0] = (unsigned char)image_buffer.at(cnt);
                cnt++;
                udp_image.at<Vec3b>(i, j)[1] = (unsigned char)image_buffer.at(cnt);
                cnt++;
                udp_image.at<Vec3b>(i, j)[2] = (unsigned char)image_buffer.at(cnt);
                cnt++;
            }
            cnt_for++;
        }
        image_cnt_past = 0;
        image_cnt_now = 0;
        image_buffer.clear();
    }
}

void MainWindow::showVideo()
{
    cv::resize(udp_image, udp_image, cv::Size(640, 360));

    int size = 10;
    Point center(qnode.robotMsg.target_x, qnode.robotMsg.target_y);
    Point v1 = center; v1.x += size;
    Point v2 = center; v2.x -= size;
    Point h1 = center; h1.y += size;
    Point h2 = center; h2.y -= size;
    Point dist = center + Point(-20, 25);
    Point rec_bias(size * 3, size * 3);

    line(udp_image, v1, v2, Scalar(0,0,255), 1, 8, 0);
    line(udp_image, h1, h2, Scalar(0,0,255), 1, 8, 0);

    std::ostringstream label_ss;
    label_ss << qnode.robotMsg.distance;
    auto label = label_ss.str();
    cv::putText(udp_image,label.c_str(),dist,cv::FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,0,255),1,cv::LINE_AA);

    if(qnode.robotMsg.isLockon)
    {
        rectangle(udp_image, center + rec_bias, center - rec_bias, Scalar(0,0,255), 1, 8 ,0);
    }

    QImage cam_image((const unsigned char *)(udp_image.data), udp_image.cols, udp_image.rows, QImage::Format_RGB888);
    ui.robot_cam->setPixmap(QPixmap::fromImage(cam_image.rgbSwapped()));

    cv::resize(udp_thermal_image, udp_thermal_image, cv::Size(480, 360));
    QImage cam_image2((const unsigned char *)(udp_thermal_image.data), udp_thermal_image.cols, udp_thermal_image.rows, QImage::Format_RGB888);
    ui.robot_cam_2->setPixmap(QPixmap::fromImage(cam_image2.rgbSwapped()));
}

double MainWindow::map(double value, double fromLow, double fromHigh, double toLow, double toHigh)
{
    double fromRange = fromHigh - fromLow;
    double toRange = toHigh - toLow;

    double valueScaled = value - fromLow;

    double mappedValue = (valueScaled * toRange) / fromRange + toLow;

    return mappedValue;
}

} // namespace irc_creative23_master_oper
