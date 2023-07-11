# ROS-Learning

## ROS 目录结构

```
WorkSpace --- 自定义的工作空间

    |--- build:编译空间，用于存放CMake和catkin的缓存信息、配置信息和其他中间文件。

    |--- devel:开发空间，用于存放编译后生成的目标文件，包括头文件、动态&静态链接库、可执行文件等。

    |--- src: 源码

        |-- package：功能包(ROS基本单元)包含多个节点、库与配置文件，包名所有字母小写，只能由字母、数字与下划线组成

            |-- CMakeLists.txt 配置编译规则，比如源文件、依赖项、目标文件

            |-- package.xml 包信息，比如:包名、版本、作者、依赖项...(以前版本是 manifest.xml)

            |-- scripts 存储python文件

            |-- src 存储C++源文件

            |-- include 头文件

            |-- msg 消息通信格式文件

            |-- srv 服务通信格式文件

            |-- action 动作格式文件

            |-- launch 可一次性运行多个节点 

            |-- config 配置信息

        |-- CMakeLists.txt: 编译的基本配置
```

**步骤**
* catkin_make
* 进入src，执行catkin_create_pkg ROSNAME roscpp rospy std_msgs
* 依次编写cpp和cmakelist文件
* catkin_make
* source ./devel/setup.bash
* rosrun ROSNAME cpp

* roscore === 是 ROS 的系统先决条件节点和程序的集合， 必须运行 roscore 才能使 ROS 节点进行通信。

* rosrun 包名 可执行文件名 === 运行指定的ROS节点

* roslaunch 包名 launch文件名 === 执行某个包下的 launch 文件

---

ROS 中的基本通信机制主要有如下三种实现策略:

* 话题通信(发布订阅模式)

* 服务通信(请求响应模式)

* 参数服务器(参数共享模式)


话题通信实现模型是比较复杂的，该模型如下图所示,该模型中涉及到三个角色:

* ROS Master (管理者)
* Talker (发布者)
* Listener (订阅者)

ROS Master 负责保管 Talker 和 Listener 注册的信息，并匹配话题相同的 Talker 与 Listener，帮助 Talker 与 Listener 建立连接，连接建立后，Talker 可以发布消息，且发布的消息会被 Listener 订阅。

## 通信策略
* 设置初始状态P
* 接收到SLAM后，赋值给P
* 接收Odom后，更新P
* 直到下次接收到SLAM，赋值给P 

imu 50Hz， slam 2Hz