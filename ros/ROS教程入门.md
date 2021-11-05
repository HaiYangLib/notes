## ROS命令行工具的使用
![在这里插入图片描述](https://img-blog.csdnimg.cn/148720bb1129412184bcdda13e0816b7.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/9abcfb7279fd4482a075c245d8c71adf.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/1830c91d5b814423a72487709e22ad3c.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/5fdf2531c61d4c6696a93e3864194c29.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/408ed7d4c6ed4246a1ca083b6d13e0a7.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)


## 创建工作空间与功能包
![在这里插入图片描述](https://img-blog.csdnimg.cn/74a705dc956f4387b3b1ed9fc9ccc6df.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/2597383c162e4bed9eeecc0f2c836ed3.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/f901d5885a2840949667c64b9b2155fc.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/76c3a2c0d99e44ec991560c5864153b0.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)

## 发布者Publisher的编程实现
![在这里插入图片描述](https://img-blog.csdnimg.cn/d0a7bdaf3b7f41e79ab5b58495990626.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)

![在这里插入图片描述](https://img-blog.csdnimg.cn/bfccc09fd33b465d845351dbd2be6819.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
* velocity_publisher.cpp 
```cpp
/**
 * 该例程将发布turtle1/cmd_vel话题，消息类型geometry_msgs::Twist
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv) {
    // ROS节点初始化
    ros::init(argc, argv, "velocity_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
    ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    // 设置循环的频率
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
        // 初始化geometry_msgs::Twist类型的消息
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.5;
        vel_msg.angular.z = 0.2;

        // 发布消息
        turtle_vel_pub.publish(vel_msg);
        ROS_INFO("Publsh turtle velocity command[%0.2f m/s, %0.2f rad/s]",
                 vel_msg.linear.x, vel_msg.angular.z);

        // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}

```
![在这里插入图片描述](https://img-blog.csdnimg.cn/b3894f02a09d4742b88d8ad97d208ce5.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/baf6e67bf2d943ddb228770a36757783.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/c92f3c09ff994d548faa570e6ab7cbbe.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
## 订阅者Subscriber的编程实现
![在这里插入图片描述](https://img-blog.csdnimg.cn/d0ababbba01e4982b1ba75498befd304.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)

![在这里插入图片描述](https://img-blog.csdnimg.cn/8266448bcec543dab29f0267247440c8.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)

![在这里插入图片描述](https://img-blog.csdnimg.cn/918a7289970a4f51bb8b989b5f3c32a2.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/6cb2a3935c84462484917d821d10169d.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
## 话题消息的定义与使用
![在这里插入图片描述](https://img-blog.csdnimg.cn/6bb53e904b9c4b7ab12a6290e4197cd7.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/813f5199c72341db995317a65075bb85.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)

```cpp
//
// Created by hhy on 2021/9/27.
//
/**
 * 该例程将发布/person_info话题，自定义消息类型learning_topic::Person
 */

#include "test_msg/Person.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  // ROS节点初始化
  ros::init(argc, argv, "person_publisher");

  // 创建节点句柄
  ros::NodeHandle n;

  // 创建一个Publisher，发布名为/person_info的topic，消息类型为learning_topic::Person，队列长度10
  ros::Publisher person_info_pub =
      n.advertise<test_msg::Person>("/person_info", 10);

  // 设置循环的频率
  ros::Rate loop_rate(1.0);

  int count = 0;
  while (ros::ok()) {
    // 初始化learning_topic::Person类型的消息
    test_msg::Person person_msg;
    person_msg.name = "Tom";
    person_msg.age = 18;
    person_msg.sex = test_msg::Person::male;

    // 发布消息
    person_info_pub.publish(person_msg);

    ROS_INFO("Publish Person Info: name:%s  age:%d  sex:%d",
             person_msg.name.c_str(), person_msg.age, person_msg.sex);

    // 按照循环频率延时
    loop_rate.sleep();
  }

  return 0;
}

```

![在这里插入图片描述](https://img-blog.csdnimg.cn/53112762145c4711af8ed9c2405c0e55.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)

```cpp
/**
 * 该例程将订阅/person_info话题，自定义消息类型learning_topic::Person
 */

#include "test_msg/Person.h"
#include <ros/ros.h>

// 接收到订阅的消息后，会进入消息回调函数
void personInfoCallback(const test_msg::Person::ConstPtr &msg) {
  // 将接收到的消息打印出来
  ROS_INFO("Subcribe Person Info: name:%s  age:%d  sex:%d", msg->name.c_str(),
           msg->age, msg->sex);
}

int main(int argc, char **argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "person_subscriber");

  // 创建节点句柄
  ros::NodeHandle n;

  // 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
  ros::Subscriber person_info_sub =
      n.subscribe("/person_info", 10, personInfoCallback);

  // 循环等待回调函数
  ros::spin();

  return 0;
}

```

![在这里插入图片描述](https://img-blog.csdnimg.cn/b447f9d39c1e489b82a9fcecdeb4657d.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/ef546bd4112c468cbe1464152cc12d65.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/071d10ecaea441c3a9c378fa1fb8c313.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
## 客户端Client的编程实现
![在这里插入图片描述](https://img-blog.csdnimg.cn/d68b16bae64b4111a18618ca102e1dbc.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
![在这里插入图片描述](https://img-blog.csdnimg.cn/6bc7410fabac479c9aeb536699fa2012.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)