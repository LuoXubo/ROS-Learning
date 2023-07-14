#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt
import time


# x, y, z = [], [], []

res = open('./res.txt', 'w+')
gt = open('./gt.txt', 'w+')


def getRes(msg):
    timestamp = msg.header.stamp
    pose = msg.pose.pose
    # x.append(float(msg.pose.pose.position.x))
    # y.append(float(msg.pose.pose.position.y))
    # z.append(float(msg.pose.pose.position.z))

    res.write('%s %f %f %f %f %f %f %f\n' % (str(timestamp), pose.position.x, pose.position.y, pose.position.z,
                                             pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))


def getGT(msg):
    timestamp = msg.header.stamp
    pose = msg.pose.pose

    gt.write('%s %f %f %f %f %f %f %f\n' % (str(timestamp), pose.position.x, pose.position.y, pose.position.z,
                                             pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))


if __name__ == "__main__":

    rospy.init_node("listener_p")
    print('Begin to listen ...')
    odom_sub = rospy.Subscriber("res", Odometry, getRes, queue_size=100)
    gt_sub = rospy.Subscriber("gt", Odometry, getGT, queue_size=100)

    rospy.spin()
