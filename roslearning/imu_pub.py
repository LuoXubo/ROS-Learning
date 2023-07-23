#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from matplotlib import pyplot as plt
import time


if __name__ == '__main__':
    rospy.init_node("imu_publisher")
    pub = rospy.Publisher("/lunar/imu", Imu, queue_size=1000)
    imu = open('./imu.csv','r')
    lines = imu.readlines()[1:]

    print('Begin to publish ...')

    for line in lines:
        items = line.split(',')
        imu_msg = Imu()
        imu_msg.header.stamp = items[0]
        imu_msg.angular_velocity.x = items[1]
        imu_msg.angular_velocity.y = items[2]
        imu_msg.angular_velocity.z = items[3]
        imu_msg.linear_acceleration.x = items[4]
        imu_msg.linear_acceleration.y = items[5]
        imu_msg.linear_acceleration.z = items[6]
        pub.publish(imu_msg)
