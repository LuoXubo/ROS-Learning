/*
    需求: 实现基本的话题通信，一方发布数据，一方接收数据，
         实现的关键点:
         1.发送方
         2.接收方
         3.数据(此处为普通文本)


    消息订阅方:
        订阅话题并打印接收到的消息

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点:命名(唯一)
        3.实例化 ROS 句柄
        4.实例化 订阅者 对象
        5.处理订阅的消息(回调函数)
        6.设置循环调用回调函数

*/
// 1.包含头文件
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <message_filters/subscriber.h>

#include <iostream>

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    std::cout << "Get imu data:\n";
    ROS_INFO("imu: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
             msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
             msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    std::cout << '\n';
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    std::cout << "Get odometry data: \n";
    ROS_INFO("odom: %f, %f, %f, %f, %f, %f", msg->pose.pose.position.x, msg->pose.pose.position.y,
             msg->pose.pose.position.z, msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    std::cout << '\n';
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    // 2.初始化 ROS 节点:命名(唯一)
    ros::init(argc, argv, "listener");
    // 3.实例化 ROS 句柄
    ros::NodeHandle nh;

    // 4.实例化 订阅者 对象
    ros::Subscriber imusub = nh.subscribe<sensor_msgs::Imu>("imu", 10, imuCallback);
    ros::Subscriber odomsub = nh.subscribe<nav_msgs::Odometry>("odom", 10, odomCallback);
    // message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/lunar/imu", 1);

    // 5.处理订阅的消息(回调函数)

    //     6.设置循环调用回调函数
    ros::spin(); // 循环读取接收的数据，并调用回调函数处理

    return 0;
}
