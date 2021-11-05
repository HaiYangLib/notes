![在这里插入图片描述](https://img-blog.csdnimg.cn/54256e9807794bd59ea16dec8f12e8cd.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)


* person_server.cpp

```cpp
//
// Created by hhy on 2021/9/28.
//
/**
 * 该例程将执行/show_person服务，服务数据类型learning_service::Person
 */

#include "test_service/Person.h"
#include <ros/ros.h>

// service回调函数，输入参数req，输出参数res
bool personCallback(test_service::Person::Request &req,
                    test_service::Person::Response &res) {
  // 显示请求数据
  ROS_INFO("Person: name:%s  age:%d  sex:%d", req.name.c_str(), req.age,
           req.sex);

  // 设置反馈数据
  res.result = "OK";

  return true;
}

int main(int argc, char **argv) {
  // ROS节点初始化
  ros::init(argc, argv, "person_server");
  // 创建节点句柄
  ros::NodeHandle n;

  // 创建一个名为/show_person的server，注册回调函数personCallback
  ros::ServiceServer person_service =
      n.advertiseService("/show_person", personCallback);

  // 循环等待回调函数
  ROS_INFO("Ready to show person informtion.");
  ros::spin();

  return 0;
}

```

* person_client.cpp
```cpp
//
// Created by hhy on 2021/9/28.
//
/**
 * 该例程将请求/show_person服务，服务数据类型learning_service::Person
 */

#include "test_service/Person.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "person_client");

  // 创建节点句柄
  ros::NodeHandle node;

  // 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
  ros::service::waitForService("/show_person");
  ros::ServiceClient person_client =
      node.serviceClient<test_service::Person>("/show_person");

  // 初始化learning_service::Person的请求数据
  test_service::Person srv;
  srv.request.name = "Tom";
  srv.request.age = 20;
  srv.request.sex = test_service::Person::Request::male;

  // 请求服务调用
  ROS_INFO("Call service to show person[name:%s, age:%d, sex:%d]",
           srv.request.name.c_str(), srv.request.age, srv.request.sex);

  person_client.call(srv);

  // 显示服务调用结果
  ROS_INFO("Show person result : %s", srv.response.result.c_str());

  return 0;
};

```

* CMakeLists.txt

```cpp
cmake_minimum_required(VERSION 3.0.2)
project(test_service)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        turtlesim
        message_generation
        )

add_service_files(FILES Person.srv)
generate_messages(DEPENDENCIES std_msgs)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(person_client src/person_client.cpp)
target_link_libraries(person_client ${catkin_LIBRARIES})
add_dependencies(person_client ${PROJECT_NAME}_gencpp)

add_executable(person_server src/person_server.cpp)
target_link_libraries(person_server ${catkin_LIBRARIES})
add_dependencies(person_server ${PROJECT_NAME}_gencpp)

```