// 组合导航节点
// 目的：接收组合导航/navlib/odom和真值/lunar/odom，
//      将两组轨迹分别保存，完成精度评定

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include <message_filters/subscriber.h>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <vector>
#include <cmath>
#include <typeinfo>
#include <fstream>

std::ofstream gtout;
std::ofstream navout;

void navCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // std::cout << "nav" << std::endl;
    float x, y, z;
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    z = msg.pose.pose.position.z;

    float qw, qx, qy, qz;
    qw = msg.pose.pose.orientation.w;
    qx = msg.pose.pose.orientation.x;
    qy = msg.pose.pose.orientation.y;
    qz = msg.pose.pose.orientation.z;

    navout << msg.header.stamp << ' ' << x << ' ' << y << ' ' << z << ' ' << qx << ' ' << qy << ' ' << qz << ' ' << qw << std::endl;
}

void gtCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // std::cout << "odom" << std::endl;
    float x, y, z;
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    z = msg.pose.pose.position.z;

    float qw, qx, qy, qz;
    qw = msg.pose.pose.orientation.w;
    qx = msg.pose.pose.orientation.x;
    qy = msg.pose.pose.orientation.y;
    qz = msg.pose.pose.orientation.z;

    gtout << msg.header.stamp << ' ' << x << ' ' << y << ' ' << z << ' ' << qx << ' ' << qy << ' ' << qz << ' ' << qw << std::endl;
}

int main(int argc, char *argv[])
{

    gtout.open("./gt.txt");
    navout.open("./nav.txt");

    setlocale(LC_ALL, "");
    ros::init(argc, argv, "nav_linstener");
    ros::NodeHandle nh;

    ros::Subscriber nav_message = nh.subscribe<nav_msgs::Odometry>(
        "navlib/odom", 1000, navCallback);
    ros::Subscriber gt_message = nh.subscribe<nav_msgs::Odometry>(
        "lunar/odom", 1000, gtCallback);

    ros::spin();

    return 0;
}