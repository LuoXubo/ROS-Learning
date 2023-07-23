#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt
import time


# 接受四组轨迹，保存为TUM文件，支持后续evo精度评定

calc = open('./calc.txt', 'w+')
gt = open('./gt.txt', 'w+')
nav = open('./nav.txt', 'w+')
slam = open('./slam.txt', 'w+')

def getcalc(msg):
    timestamp = float(str(msg.header.stamp))
    pose = msg.pose.pose

    calc.write('%s %f %f %f %f %f %f %f\n' % (str(timestamp/1e9), pose.position.x, pose.position.y, pose.position.z,
                                             pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))


def getgt(msg):
    timestamp = float(str(msg.header.stamp))
    pose = msg.pose.pose

    gt.write('%s %f %f %f %f %f %f %f\n' % (str(timestamp/1e9), pose.position.x, pose.position.y, pose.position.z,
                                             pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

def getnav(msg):
    timestamp = float(str(msg.header.stamp))
    pose = msg.pose.pose

    nav.write('%s %f %f %f %f %f %f %f\n' % (str(timestamp/1e9), pose.position.x, pose.position.y, pose.position.z,
                                             pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

def getslam(msg):
    timestamp = float(str(msg.header.stamp))
    pose = msg.pose.pose

    slam.write('%s %f %f %f %f %f %f %f\n' % (str(timestamp/1e9), pose.position.x, pose.position.y, pose.position.z,
                                             pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))


if __name__ == "__main__":

    rospy.init_node("listener_p")
    print('Begin to listen ...')
    calc_sub = rospy.Subscriber("/calc/odom", Odometry, getcalc, queue_size=1000)
    gt_sub = rospy.Subscriber("/lunar/odom", Odometry, getgt, queue_size=1000)
    nav_sub = rospy.Subscriber("/navlib/odom", Odometry, getnav, queue_size=1000)
    slam_sub = rospy.Subscriber("/slam/odom", Odometry, getslam, queue_size=1000)

    rospy.spin()
