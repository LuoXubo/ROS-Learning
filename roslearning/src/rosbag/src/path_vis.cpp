#include "ros/ros.h"
#include "ros/time.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <cmath>

void calcCallback(const nav_msgs::Odometry &msg);
void slamCallback(const nav_msgs::Odometry &msg);
void navCallback(const nav_msgs::Odometry &msg);
void gtCallback(const nav_msgs::Odometry &msg);
ros::Publisher gt_line, nav_line, slam_line, calc_line;

visualization_msgs::Marker calc_path, slam_path, nav_path, gt_path;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_vis");
    ros::NodeHandle nh;
    


    // tf::TransformBroadcaster br;
    // tf::Transform transform;
    // tf::Quaternion q(0,0,0,1);
    // while (1)
    // {
    //     transform.setOrigin(tf::Vector3(0,0,i));

    //     // tf::Quaternion q(0,0,0,1);  
    //     transform.setRotation(q);
    //     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "gt"));

    // }

    calc_path.color.r = 1.0;
    calc_path.color.a = 1.0;
    calc_path.type = visualization_msgs::Marker::LINE_STRIP;
    calc_path.header.frame_id = "odom";
    calc_path.ns = "points_and_lines";
    calc_path.action = visualization_msgs::Marker::ADD;
    calc_path.pose.orientation.w = 1.0;
    calc_path.scale.x = 0.005;

    slam_path.color.b = 1.0;
    slam_path.color.a = 1.0;
    slam_path.type = visualization_msgs::Marker::LINE_STRIP;
    slam_path.header.frame_id = "odom";
    slam_path.ns = "points_and_lines";
    slam_path.action = visualization_msgs::Marker::ADD;
    slam_path.pose.orientation.w = 1.0;
    slam_path.scale.x = 0.005;

    nav_path.color.g = 1.0;
    nav_path.color.a = 1.0;
    nav_path.type = visualization_msgs::Marker::LINE_STRIP;
    nav_path.header.frame_id = "odom";
    nav_path.ns = "points_and_lines";
    nav_path.action = visualization_msgs::Marker::ADD;
    nav_path.pose.orientation.w = 1.0;
    nav_path.scale.x = 0.005;

    gt_path.color.r = 1.0;
    gt_path.color.g = 1.0;
    gt_path.color.a = 1.0;
    gt_path.type = visualization_msgs::Marker::LINE_STRIP;
    gt_path.header.frame_id = "odom";
    gt_path.ns = "points_and_lines";
    gt_path.action = visualization_msgs::Marker::ADD;
    gt_path.pose.orientation.w = 1.0;
    gt_path.scale.x = 0.005;

    calc_line =
        nh.advertise<visualization_msgs::Marker>("calc_path", 1000);
    slam_line =
        nh.advertise<visualization_msgs::Marker>("slam_path", 1000);
    nav_line =
        nh.advertise<visualization_msgs::Marker>("nav_path", 1000);
    gt_line =
        nh.advertise<visualization_msgs::Marker>("gt_path", 1000);

    // ImuIntegrator *imu_integrator = new ImuIntegrator(line, odom_line);

    ros::Subscriber calc_sub = nh.subscribe(
        "/calc/odom", 1000, calcCallback);

    ros::Subscriber slam_sub = nh.subscribe(
        "/slam/odom", 1000, slamCallback);

    ros::Subscriber nav_sub = nh.subscribe(
        "/navlib/odom", 1000, navCallback);
    ros::Subscriber gt_sub = nh.subscribe(
        "/lunar/odom", 1000, gtCallback);

    ros::spin();
}

void calcCallback(const nav_msgs::Odometry &msg)
{
    geometry_msgs::Point p;
    p.x = msg.pose.pose.position.x;
    p.y = msg.pose.pose.position.y;
    p.z = msg.pose.pose.position.z;
    calc_path.points.push_back(p);
    calc_line.publish(calc_path);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
    transform.setOrigin(tf::Vector3(p.x,p.y,p.z));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "calc"));
}
void slamCallback(const nav_msgs::Odometry &msg)
{
    geometry_msgs::Point p;
    p.x = msg.pose.pose.position.x;
    p.y = msg.pose.pose.position.y;
    p.z = msg.pose.pose.position.z;
    slam_path.points.push_back(p);
    slam_line.publish(slam_path);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
    transform.setOrigin(tf::Vector3(p.x,p.y,p.z));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "slam"));
}
void navCallback(const nav_msgs::Odometry &msg)
{
    geometry_msgs::Point p;
    p.x = msg.pose.pose.position.x;
    p.y = msg.pose.pose.position.y;
    p.z = msg.pose.pose.position.z;
    nav_path.points.push_back(p);
    nav_line.publish(nav_path);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
    transform.setOrigin(tf::Vector3(p.x,p.y,p.z));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "nav"));
}
void gtCallback(const nav_msgs::Odometry &msg)
{
    geometry_msgs::Point p;
    p.x = msg.pose.pose.position.x;
    p.y = msg.pose.pose.position.y;
    p.z = msg.pose.pose.position.z;
    gt_path.points.push_back(p);
    

    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "gt"));
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
    transform.setOrigin(tf::Vector3(p.x,p.y,p.z));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "gt"));
    
    gt_line.publish(gt_path);
}