#!/usr/bin/env python2
import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu 
from matplotlib import pyplot as plt
import time

xs, ys, zs = [], [], []
timestamps = []

def getImu(msg):
    timestamp = float(str(msg.header.stamp))/1e9
    linear_acceleration = msg.linear_acceleration
    angular_velocity = msg.angular_velocity
    orientation = msg.orientation
    timestamps.append(timestamp)
    xs.append(linear_acceleration.x)
    ys.append(linear_acceleration.y)
    zs.append(linear_acceleration.z)
    # 使用plt分别用三个子图实时可视化linear_acceleration的x,y,z
    plt.ion()
    # plt.figure()
    plt.subplot(311)
    plt.plot(timestamps, xs, 'r-',label='x')
    plt.xlabel('time')
    plt.ylabel('linear_acceleration_x')
    plt.subplot(312)
    plt.plot(timestamps, ys, 'g-',label='y')
    plt.xlabel('time')
    plt.ylabel('linear_acceleration_y')
    plt.subplot(313)
    plt.plot(timestamps, zs, 'b-',label='z')
    plt.xlabel('time')
    plt.ylabel('linear_acceleration_z')
    plt.pause(0.0000001)
    plt.ioff()
    # plt.show()
    # print(linear_acceleration)
    # timestamps只保留最新的100条数据
    if len(timestamps) > 25:
        timestamps.pop(0)
        xs.pop(0)
        ys.pop(0)
        zs.pop(0)

    
    


if __name__ == "__main__":

    rospy.init_node("imuListener")
    print('Begin to listen ...')
    # imu_sub = rospy.Subscriber("/lunar/imu", Odometry, getslam, queue_size=1000)
    imu_sub = rospy.Subscriber("/lunar/imu", Imu, getImu, queue_size=1000)


    rospy.spin()
