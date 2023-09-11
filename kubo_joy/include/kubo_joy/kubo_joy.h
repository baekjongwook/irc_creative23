#include "ros/ros.h"  //ROS 기본 헤더파일
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "kubo_msgs/humo.h"
#include "iostream"

#define MAX_F_FLIPPER 130
#define MIN_F_FLIPPER -90
#define MAX_B_FLIPPER 100
#define MIN_B_FLIPPER -90

sensor_msgs::Joy joy_mani;
kubo_msgs::humo humoMsg;

void msgCallback(const sensor_msgs::Joy::ConstPtr &joy);

ros::Publisher pub_joy;
ros::Subscriber sub;

bool init_done=false;

/*joystick data_flipper*/
float _flipper_pos[4]={130,130,90,90};
bool flipper_auto=false;

/*joystick data_mani*/

std::string connect;
bool joy_mode=false;

bool in_once=true;
bool waypoint_btn_once = true;

bool x_btn_once = true;
bool o_btn_once = true;

bool flipper_init_trigger=false;

float
leftStickY=0,
leftStickX=0,
rightStickY=0,
rightStickX=0,
l2,
r2;

int
arrowsX=0,
arrowsY=0,
buttonSq=0,
buttonX=0,
buttonO=0,
buttonTr=0,
l1=0,
r1=0,
buttonShare=0;

int buttonOption=0;
int buttonTouch=0;
int yawturn=0;
int buttonCenter=0;
int buttonLeftJoy=0;
int buttonRightJoy=0;

float rightStickX_, rightStickY_;
float axisL2,axisR2;

int buttonModeChange=0;
int ModeChange=0;

float data_sum=0;
