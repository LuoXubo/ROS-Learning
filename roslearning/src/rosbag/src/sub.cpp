#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include <message_filters/subscriber.h>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <vector>
#include <cmath>

nav_msgs::Odometry pre_odom; // 当前状态量
bool isFirst = true;         // 判断是否完成初始设定
double deltaT;               // 时间间隔

Eigen::Vector3d zero(0, 0, 0);
Eigen::Vector3d pos = zero;                          // 位置
Eigen::Matrix3d orien = Eigen::Matrix3d::Identity(); // 姿态 旋转矩阵表示
Eigen::Vector3d w = zero;                            // 角速度
Eigen::Vector3d v = zero;                            // 线速度
Eigen::Vector3d gravity;                             // 重力加速度

void odom_update(const sensor_msgs::Imu::ConstPtr &msg);
void odom_init();
void matrix2odom();
void odom2matrix();
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "listener");

    ros::NodeHandle nh;

    // 初始化当前状态
    odom_init();

    ros::Subscriber imusub = nh.subscribe<sensor_msgs::Imu>("imu", 100, imuCallback);
    ros::Subscriber odomsub = nh.subscribe<nav_msgs::Odometry>("odom", 100, odomCallback);
    // message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/lunar/imu", 1);

    ros::spin(); // 循环读取接收的数据，并调用回调函数处理

    return 0;
}

void odom_update(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (isFirst)
    {
        pre_odom.header.stamp = msg->header.stamp;
        deltaT = 0;
        isFirst = false;
        gravity[0] = msg->linear_acceleration.x;
        gravity[1] = msg->linear_acceleration.y;
        gravity[2] = msg->linear_acceleration.z;
    }
    else
    {
        deltaT = (msg->header.stamp - pre_odom.header.stamp).toSec();
        if (deltaT < 0)
        {
            std::cout << "Error! deltaT < 0 !\n\n";
        }
        pre_odom.header.stamp = msg->header.stamp;

        // 计算方向
        auto angular = msg->angular_velocity;
        w << angular.x, angular.y, angular.z;
        // 基于旋转矩阵表示方法
        Eigen::Matrix3d B;
        B << 0, -angular.z * deltaT, angular.y * deltaT,
            angular.z * deltaT, 0, -angular.x * deltaT,
            -angular.y * deltaT, angular.x * deltaT, 0;
        // 欧拉法
        double sigma = std::sqrt(std::pow(angular.x, 2) + std::pow(angular.y, 2) + std::pow(angular.z, 2)) * deltaT;
        // 罗德里格斯公式
        orien = orien * (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B -
                         ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);

        // 计算位置
        auto linear = msg->linear_acceleration;
        Eigen::Vector3d acc_l(linear.x, linear.y, linear.z); // imu坐标系下的加速度
        Eigen::Vector3d acc_g = orien * acc_l;               // 转化到里程计坐标系下的加速度
        // Eigen::Vector3d acc(msg.x - gravity[0], msg.y - gravity[1], msg.z -
        // gravity[2]);
        v = v + deltaT * (acc_g - gravity); // 积分得到速度
        pos = pos + deltaT * v;             // 积分得到位置
    }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // std::cout << "Get imu data:\n";
    // ROS_INFO("imu: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
    //          msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
    //          msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    // std::cout << '\n';

    odom_update(msg);
    matrix2odom();
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // std::cout << "Get odometry data: \n";
    // ROS_INFO("odom: %f, %f, %f, %f, %f, %f", msg->pose.pose.position.x, msg->pose.pose.position.y,
    //          msg->pose.pose.position.z, msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    // std::cout << '\n';

    pre_odom.header.stamp = msg->header.stamp;
    pre_odom.header.frame_id = msg->header.frame_id;

    pre_odom.pose.pose.position.x = msg->pose.pose.position.x;
    pre_odom.pose.pose.position.y = msg->pose.pose.position.y;
    pre_odom.pose.pose.position.z = msg->pose.pose.position.z;

    pre_odom.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    pre_odom.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    pre_odom.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    pre_odom.pose.pose.orientation.w = msg->pose.pose.orientation.w;

    odom2matrix(); // update position and orientation matrix
}

void odom_init()
{
    pre_odom.header.stamp = ros::Time::now();
    matrix2odom();
}

void matrix2odom()
{
    // 位置
    pre_odom.pose.pose.position.x = pos(0);
    pre_odom.pose.pose.position.y = pos(1);
    pre_odom.pose.pose.position.z = pos(2);
    // 姿态 四元数
    pre_odom.pose.pose.orientation.x = (orien(2, 1) - orien(1, 2)) / 4;
    pre_odom.pose.pose.orientation.y = (orien(0, 2) - orien(2, 0)) / 4;
    pre_odom.pose.pose.orientation.z = (orien(1, 0) - orien(0, 1)) / 4;
    pre_odom.pose.pose.orientation.w = std::sqrt(1 + orien(0, 0) + orien(1, 1) + orien(2, 2)) / 2;
    // 线速度
    pre_odom.twist.twist.linear.x = v(0);
    pre_odom.twist.twist.linear.y = v(1);
    pre_odom.twist.twist.linear.z = v(2);
    // 角速度
    pre_odom.twist.twist.angular.x = w(0);
    pre_odom.twist.twist.angular.y = w(1);
    pre_odom.twist.twist.angular.z = w(2);
}

void odom2matrix()
{
    // 位置
    pos(0) = pre_odom.pose.pose.position.x;
    pos(1) = pre_odom.pose.pose.position.y;
    pos(2) = pre_odom.pose.pose.position.z;
    // 姿态
    Eigen::Quaterniond quat;
    quat.x() = pre_odom.pose.pose.orientation.x;
    quat.y() = pre_odom.pose.pose.orientation.y;
    quat.z() = pre_odom.pose.pose.orientation.z;
    quat.w() = pre_odom.pose.pose.orientation.w;
    orien = quat.normalized().toRotationMatrix();
}