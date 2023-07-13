import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt

x = []
y = []
z = []

fig = plt.figure()
ax = fig.gca(projection='3d')


def doMsg(msg):
    print(msg.data)
    # # rospy.loginfo("I heard:%s",msg.data)
    # x.append(float(msg.data.pose.pose.position.x))
    # y.append(float(msg.data.pose.pose.position.y))
    # z.append(float(msg.data.pose.pose.position.z))
    # ax.plot(x, y, z, c='r')
    # plt.show()
    

if __name__ == "__main__":

    rospy.init_node("listener_p")

    sub = rospy.Subscriber("res",Odometry,doMsg,queue_size=100)

    rospy.spin()
