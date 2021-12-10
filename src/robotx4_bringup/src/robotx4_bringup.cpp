#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/String.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/String.h>

#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <sys/time.h>

#include <iostream>
#include <fstream>
#include <sstream>

#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_bridge/socketcan_to_topic.h>

#include <can_msgs/Frame.h>
#include <socketcan_interface/socketcan.h>
 using namespace std;

/*全局变量区*/
float WHEEL_RATIO = 70; //电机减速比
float WHEEL_L = 0.740; //左右轮子的轮间距
float WHEEL_D = 0.400; //左右轮子的直径
float WHEEL_R = WHEEL_D/2.0; //左右轮子的半径
float WHEEL_PI = 3.141593; //π值
unsigned int speed_0ffset = 150;  //电机转速偏移10000转
float position_x=0,position_y=0,position_w=0;  //定义机器人底盘中里程计的初始位置，x，y方向位移和航向角

int rec_motor1_rpm = 0;//主控从底盘接收到的电机1的实时转速
int rec_motor2_rpm = 0;//主控从底盘接收到的电机2的实时转速
int rec_motor3_rpm = 0;//主控从底盘接收到的电机3的实时转速
int rec_motor4_rpm = 0;//主控从底盘接收到的电机4的实时转速
// char buffer[33]={0};  //整型变量转为十六进制字符对应的数组


/*电机数据结构体*/
typedef struct 
{
    unsigned int  Node_ID; //电机终端（ID）号
    int rpm; //电机转速，有正有负
    int DEC; //1000为四倍频下的电机编码器分辨率（驱动器指定公式）
    // string str; //指向电机DEC转为十六进制字符数组的指针
}motor_measure;
motor_measure motor_send[4];//声明电机转速发送数组
// motor_measure motor_rec[4];//声明电机转速接收数组

/*函数声明区*/
// char *dectohex(int num);
// char * inttohex(int num);
void send_can_message(motor_measure *motor_send, int n);
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr&msg);
void rec_can_message_callback(const can_msgs::Frame::ConstPtr&rec_can);
void calculate_and_publish_odometry(void);
ros::Subscriber command_sub, can_message_rec;
ros::Publisher odom_pub;

int main(int argc, char**argv)
{
    ros::init(argc, argv, "robotx4");
    ros::NodeHandle n("~");
    std::string pub_odom_topic,sub_cmdvel_topic;
    n.param<std::string>("sub_cmdvel_topic",sub_cmdvel_topic,"/cmd_vel");
    n.param<std::string>("pub_odom_topic", pub_odom_topic, "/odom");
    //订阅cmd_vel话题
    command_sub = n.subscribe(sub_cmdvel_topic,10,cmd_vel_callback);
    //定义需要发布的/odom话题
    odom_pub = n.advertise<nav_msgs::Odometry>(pub_odom_topic,20);
    ros::spin();
    while(ros::ok())
    {
        calculate_and_publish_odometry();
        ros::spinOnce();
    }    
}

/*利用cmd_vel回调函数计算电机转速（转/分），并通过can口发送到底盘*/
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr&msg)
{
    float speed_x,speed_w;                     //机器人x方向移动速度（前进速度）以及z轴转速（角速度）
    float v1=0,v2=0,v3=0,v4=0;               //机器人四个轮子的线速度初始化定义

    speed_x = msg->linear.x;
    speed_w = msg->angular.z;

    /*计算机器人每个轮子的线速度（圆周速度），左前为1，左后为2，右后为3，右前为4；即 1  4
                                                                                                                                                                                        2  3
    */
    v1 = speed_x-0.5*WHEEL_L*speed_w;
    v2 = v1;
    v4 = - (speed_x+0.5*WHEEL_L*speed_w) ;
    v3 = v4;
    /*根据圆周速度速度公式V=πdn计算轮子的转速 πd（周长）；n:转速（转/分）,此处计算得到n，单位为：转/秒 （rps）*/
    v1 = v1/(2.0*WHEEL_R*WHEEL_PI);
    v2 = v2/(2.0*WHEEL_R*WHEEL_PI);
    v3 = v3/(2.0*WHEEL_R*WHEEL_PI);
    v4 = v4/(2.0*WHEEL_R*WHEEL_PI);

   /*由于存在减速比，所以轮子实际转动量是电机输出轴的转速比的倍数，此处的转速单位为：转/分（rpm） */
    v1 = v1*WHEEL_RATIO*60;
    v2 = v2*WHEEL_RATIO*60;
    v3 = v3*WHEEL_RATIO*60;
    v4 = v4*WHEEL_RATIO*60;
   /*将四个电机的转速通过can发送到机器人底盘底盘*/
    motor_send[0].rpm = v1 + speed_0ffset;
    motor_send[1].rpm = v2 + speed_0ffset;
    motor_send[2].rpm = v3 + speed_0ffset;
    motor_send[3].rpm = v4 + speed_0ffset;
    /*can通信每个电机对应的地址*/
    motor_send[0].Node_ID = 601;
    motor_send[1].Node_ID = 602;
    motor_send[2].Node_ID = 603;
    motor_send[3].Node_ID = 604;
    /*每个电机的DEC计算*/
    // motor_r[0].DEC = motor_r[0].rpm*512*10000/1875;
    motor_send[0].DEC = motor_send[0].rpm*512*10000/1875;
    motor_send[1].DEC = motor_send[1].rpm*512*10000/1875;
    motor_send[2].DEC = motor_send[2].rpm*512*10000/1875;
    motor_send[3].DEC = motor_send[3].rpm*512*10000/1875;
    /*因为十六进制传输，所以将每个电机的DEC转换为十六进制字符数组*/
    // motor_r[0].str = dectohex(motor_r[0].DEC);
    // motor_r[1].str = dectohex(motor_r[1].DEC);
    // motor_r[2].str = dectohex(motor_r[2].DEC);
    // motor_r[3].str = dectohex(motor_r[3].DEC);

    // ROS_INFO("%s%s%s%s",motor_r[0].str.c_str(),motor_r[1].str.c_str(),motor_r[2].str.c_str(),motor_r[3].str.c_str());

    send_can_message(motor_send, 4);  //4表示结构体数组元素数量

}

 /*发送函数：将四个电机的转速通过can发送的机器人底盘*/
 /*里程计计算函数：通过获取底盘电机的实时转速，计算机器人轮子的线速度，推算里程计信息 
     方法原理：利用电机的实时转速推到里程计位移信息*/
void calculate_and_publish_odometry(void)
{
    //订阅底盘received——msg话题
    while(ros::ok())
    {
            ros::NodeHandle can_rec;
            ros::Subscriber rec_can_message;
            rec_can_message = can_rec.subscribe("/received_message",10,rec_can_message_callback);
            ros::spin();
    }
    float v1=0,v2=0,v3=0,v4=0;
    float linear_x,linear_y,linear_w;
    //根据电机转速推算机器人轮子的实时速度
    v1 = rec_motor1_rpm/WHEEL_RATIO/60.0*WHEEL_R*WHEEL_PI*2;
    v2 = rec_motor2_rpm/WHEEL_RATIO/60.0*WHEEL_R*WHEEL_PI*2;
    v3 = rec_motor3_rpm/WHEEL_RATIO/60.0*WHEEL_R*WHEEL_PI*2;
    v4 = rec_motor4_rpm/WHEEL_RATIO/60.0*WHEEL_R*WHEEL_PI*2;

    //计算机器人底盘的运动速度和转动角度
    linear_x = 0.25*v1 + 0.25*v2 + 0.25*v3 +0.25*v4;
    linear_y = 0;
    linear_w = ((0.5*v3+0.5*v4)-(0.5*v1+0.5*v2))/float(WHEEL_L);

    float dt =0.02; //采样间隔20ms
    //求解机器人底盘里程计x方向位移量，y方向位移量，z轴转到量
    float delta_position_x = (linear_x * cos(position_w) - linear_y * sin(position_w)) *dt;
    float delta_position_y = (linear_x * sin(position_w) + linear_y * cos(position_w)) *dt; 
    float delta_position_w = linear_w * dt;

    position_x  += delta_position_x;
    position_y  += delta_position_y; 
    position_w += delta_position_w;

    ros::Rate r(1.0);
    static tf::TransformBroadcaster odom_broadcaster; //定义tf对象
    geometry_msgs::TransformStamped odom_trans;  //创建一个tf发布需要使用的TransformStamped类型消息
    geometry_msgs::Quaternion odom_quat; //四元数变量
    nav_msgs::Odometry odom;  //定义里程计对象

    //里程计的航向角（偏航角）需要转换成四元数才可以发布
    odom_quat = tf::createQuaternionMsgFromYaw(position_w);
    //载入坐标（tf）变换时间戳
    odom_trans.header.stamp = ros::Time::now();
    //发布坐标变换的父子坐标系
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_  link";
    //tf位置数据：x，y，z方向
    odom_trans.transform.translation.x = position_x;
    odom_trans.transform.translation.y = position_y;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = odom_quat;

    //载入里程计时间戳
    odom.header.stamp = ros::Time::now();
    //里程计的父子坐标系
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    //里程计位置数据：x，y，z方向
    odom.pose.pose.position.x = position_x;
    odom.pose.pose.position.y = position_y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_quat;

    //载入机器人线速度和角速度
    odom.twist.twist.linear.x = linear_x;
    odom.twist.twist.linear.y = linear_y;
    odom.twist.twist.linear.z = linear_w;

    //发布里程计
    odom_pub.publish(odom);

    r.sleep();
}

/*通过ros_canopen，主控将速度信息发送到底盘*/
void send_can_message(motor_measure *motor_send, int n)
{
    
    while(ros::ok())
    {
        ROS_INFO("start send can_message");
        setlocale(LC_ALL,"");
        uint8_t transition[8];//中间传递转速变量数组
        can_msgs::Frame can_frame;
        memset(&transition,0,sizeof(transition));//初始化数组元素为0
        
        for(uint8_t i=0; i<4; i++)
        {
            can_frame.id = motor_send[i].Node_ID;
            can_frame.is_extended = false;
            can_frame.is_rtr = false;
            can_frame.is_error = false;
            can_frame.dlc = 8;//有效数据长度
            transition[0] = 0x23;
            transition[1] = 0xFF;
            transition[2] = 0x60;
            transition[3] = 0x00;
            transition[4] = motor_send[i].DEC & 0x000000FF;
            transition[5] = (motor_send[i].DEC & 0x0000FF00) >> 8;
            transition[6] = (motor_send[i].DEC & 0x00FF0000) >> 16;
            transition[7] = (motor_send[i].DEC & 0xFF000000) >> 24;
            for(uint8_t j =0; j<can_frame.dlc; j++)
            {
                can_frame.data[j] = transition[j];
            } 
            // ros::NodeHandle roscan("~");
            ros::NodeHandle roscan;
            ros::Publisher roscan_send_message;
            roscan_send_message = roscan.advertise<can_msgs::Frame>("sent_messages",100);
            roscan_send_message.publish(can_frame);
            ROS_INFO("has sent messages to motor 1-4");
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }
    }
}

/*通过ros_canopen，主控接收底盘发送的电机转速信息*/
void rec_can_message_callback(const can_msgs::Frame::ConstPtr& rec_can)
{
    uint motor1_rec_id = 581;//电机1接收ID
    uint motor2_rec_id = 582;//电机2接收ID
    uint motor3_rec_id = 583;//电机3接收ID
    uint motor4_rec_id = 584;//电机4接收ID  
    while(ros::ok())
    {
        ROS_INFO("start receive can_message");
        setlocale(LC_ALL,"");
        uint8_t transition[8];//中间传递转速变量数组
        can_msgs::Frame can_frame;
        memset(&transition,0,sizeof(transition));//初始化数组元素为0
        for(uint8_t i =0; i<8; i++)
        {
            transition[i] = rec_can->data[i];
        } 
        if (rec_can->id != motor1_rec_id && rec_can->id != motor2_rec_id && rec_can->id != motor3_rec_id && rec_can->id != motor4_rec_id)
        {
            ROS_INFO("An error occurred during reception");
        }
        else if (rec_can->id == motor1_rec_id)
        {
            int motor1_d1, motor1_d2,motor1_d3;
            motor1_d1 = transition[5];
            motor1_d2 = transition[6];
            motor1_d3 = transition[7];
            rec_motor1_rpm = (motor1_d3 << 24)|(motor1_d2 << 16)|(motor1_d1 << 8)|transition[4];
        }
        else if (rec_can->id == motor2_rec_id)
        {
            int motor2_d1, motor2_d2,motor2_d3;
            motor2_d1 = transition[5];
            motor2_d2 = transition[6];
            motor2_d3 = transition[7];
            rec_motor2_rpm = (motor2_d3 << 24)|(motor2_d2 << 16)|(motor2_d1 << 8)|transition[4];
        }
        else if (rec_can->id == motor3_rec_id)
        {
            int motor3_d1, motor3_d2,motor3_d3;
            motor3_d1 = transition[5];
            motor3_d2 = transition[6];
            motor3_d3 = transition[7];
            rec_motor3_rpm = (motor3_d3 << 24)|(motor3_d2 << 16)|(motor3_d1 << 8)|transition[4];
        }
        else if (rec_can->id == motor4_rec_id)
        {
            int motor4_d1, motor4_d2,motor4_d3;
            motor4_d1 = transition[5];
            motor4_d2 = transition[6];
            motor4_d3 = transition[7];
            rec_motor4_rpm = (motor4_d3 << 24)|(motor4_d2 << 16)|(motor4_d1 << 8)|transition[4];
        }
    }
}
//十进制转十六进制（十六进制字母必须为大写，且在终端调试时显示的是对应字符的ASCII值）
// char * inttohex(int num)
// {
//     static int i = 0;
//     if(num < 16)
//     {
//         if(num<10)
//         buffer[i] = num +'0';
//         else 
//         buffer[i] = num - 10 + 'A';
//         buffer[i+1] = '\0';
//     }
//     else
//     {
//         inttohex(num/16);
//         i++;
//         num%=16;
//         if(num<10)
//         buffer[i]=num +'0';
//         else
//         {
//             buffer[i]=num -10 +'A';
//         }
//         return (buffer);
//     }   
// }

// char *dectohex(int num)
// {
//     sprintf(buffer,"%8x",num);
//     return (buffer);
// }
/*主控通过订阅received_messages话题，接收底盘发送的转速信息 */




