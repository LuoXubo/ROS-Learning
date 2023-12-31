#include "ros/ros.h"
#include "std_msgs/String.h" //普通文本类型的消息
# include "sensor_msgs/Imu.h"
#include <sstream>

int main(int argc, char  *argv[])
{   
    //设置编码
    setlocale(LC_ALL,"");

    //2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    ros::init(argc,argv,"imu_talker");
    //3.实例化 ROS 句柄
    ros::NodeHandle nh;//该类封装了 ROS 中的一些常用功能

    //4.实例化 发布者 对象
    //泛型: 发布的消息类型
    //参数1: 要发布到的话题
    //参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("imu",10);

    //5.组织被发布的数据，并编写逻辑发布数据
    //数据(动态组织)
    int count = 0; //消息计数器

    //逻辑(一秒10次)
    ros::Rate r(5);

    //节点不死
    while (ros::ok())
    {
        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "base_link";
        
        imu_data.orientation.x = 0;
        imu_data.orientation.y = 0;
        imu_data.orientation.z = 0;
        imu_data.orientation.w = 1;

        imu_data.linear_acceleration.x = 0.01;
        imu_data.linear_acceleration.y = 0.02;
        imu_data.linear_acceleration.z = 0.03;

        imu_data.angular_velocity.x = 0.05;
        imu_data.angular_velocity.y = 0.06;
        imu_data.angular_velocity.z = 0.07;

        //发布消息
        pub.publish(imu_data);

        //根据前面制定的发送贫频率自动休眠 休眠时间 = 1/频率；
        r.sleep();
        count++;//循环结束前，让 count 自增
        //暂无应用
        ros::spinOnce();
    }


    return 0;
}
