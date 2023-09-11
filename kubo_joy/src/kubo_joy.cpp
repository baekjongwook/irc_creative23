#include "../include/kubo_joy/kubo_joy.h"
#include "sensor_msgs/Joy.h"
#define MAX_VEL 0.3
#define MAX_ANG 0.3

int main(int argc, char **argv) //c++ 의 기본 함수형태
{
    ros::init(argc, argv, "kubo_joy"); //노드명 초기화
    ros::NodeHandle nh; //ROS시스템과 통신을 위한 노드핸들 선언
    ros::NodeHandle nh_;

    pub_joy=nh.advertise<kubo_msgs::humo>("humo_joy",100);
    sub = nh.subscribe("joy", 100, msgCallback);
    nh_.getParam("connect",connect);

    ros::Rate loop_rate(100);
    while(ros::ok()) //ros 가 활성화되면
    {

      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}

void msgCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    humoMsg.leftStickX = joy->axes[0];
    humoMsg.leftStickY = joy->axes[1];
    humoMsg.rightStickX = joy->axes[4];
    humoMsg.rightStickY = joy->axes[3];
    humoMsg.axisL2 = joy->axes[2];
    humoMsg.axisR2 = joy->axes[5];
    humoMsg.arrowsX = joy->axes[6];
    humoMsg.arrowsY = joy->axes[7];

    humoMsg.buttonX = joy->buttons[0];
    humoMsg.buttonO = joy->buttons[1];
    humoMsg.buttonTr = joy->buttons[2];
    humoMsg.buttonSq = joy->buttons[3];
    humoMsg.l1 = joy->buttons[4];
    humoMsg.r1 = joy->buttons[5];
    humoMsg.l2 = joy->buttons[6];
    humoMsg.r2 = joy->buttons[7];
    humoMsg.buttonShare = joy->buttons[8];
    humoMsg.buttonOption= joy->buttons[9];
    humoMsg.buttonCenter = joy->buttons[10];
    humoMsg.buttonLeftJoy= joy->buttons[11];
    humoMsg.buttonRightJoy = joy->buttons[12];
    humoMsg.buttonTouch= joy->buttons[13];

    pub_joy.publish(humoMsg);

}


