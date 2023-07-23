#!/usr/bin/env python2
import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from matplotlib import pyplot as plt
import time


if __name__ == '__main__':
    rospy.init_node("odom_publisher")
    pub = rospy.Publisher("/lunar/odom", Odometry, queue_size=1000)
    rate = rospy.Rate(1)

    gt = open('./gt.csv','r')
    lines = gt.readlines()[1:]

    print('Begin to publish ...')

    for line in lines:
        items = line.split(',')
        odom_msg = Odometry()

        odom_msg.header.stamp = rospy.Time(float(items[0])/1e9)
        odom_msg.pose.pose.position.x = float(items[1])
        odom_msg.pose.pose.position.y = float(items[2])
        odom_msg.pose.pose.position.z = float(items[3])

        odom_msg.pose.pose.orientation.w = float(items[4])
        odom_msg.pose.pose.orientation.x = float(items[5])
        odom_msg.pose.pose.orientation.y = float(items[6])
        odom_msg.pose.pose.orientation.z = float(items[7])

        pub.publish(odom_msg)
        print(odom_msg)

        rate.sleep()
