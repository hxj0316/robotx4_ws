#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/String.h>

#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>

#include <std_msgs/String.h>
#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_bridge/socketcan_to_topic.h>

#include <can_msgs/Frame.h>
#include <socketcan_interface/socketcan.h>

void send_can_message(int motor_1,int motor_2,int motor_3,int motor_4)
{
    
    while(ros::ok())
    {
    ROS_INFO("start send can_message");
    setlocale(LC_ALL,"");
    uint8_t transition[8];//中间传递转速变量数组
    can_msgs::Frame can_frame;
    memset(&transition,0,sizeof(transition));//初始化数组元素为0

    transition[0]=motor_1 >> 8;
    transition[1]=motor_1 & 0xFF;
    transition[2]=motor_2 >> 8;
    transition[3]=motor_2 & 0xFF;
    transition[4]=motor_3 >> 8;
    transition[5]=motor_3 & 0xFF;
    transition[6]=motor_4 >> 8;
    transition[7]=motor_4 & 0xFF8;

    can_frame.id=0x07ff1234;//需要修改
    can_frame.is_extended = true;
    can_frame.is_rtr = false;
    can_frame.is_error = false;
    can_frame.dlc = 8;//有效数据长度
    for(uint8_t i =0; i<can_frame.dlc; i++)
    {
        can_frame.data[i] = transition[i];
    }
    ros::NodeHandle roscan("~");
    ros::Publisher roscan_send_message;
    roscan_send_message = roscan.advertise<can_msgs::Frame>("sent_messages",100);
    roscan_send_message.publish(can_frame);
    ROS_INFO("has sent messages to motor 1-4");
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    }
}

int main(int argc, char**argv)
{
    ros::init(argc, argv, "robotx4");
    ros::NodeHandle n("~");
    send_can_message(100,200,300,400);
}