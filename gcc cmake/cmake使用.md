
# cmake使用
## [CMakeLists.txt中常用库文件的查找和链接](https://zhuanlan.zhihu.com/p/73373016)
## [cmake入门实战](https://www.hahack.com/codes/cmake/)
## cmake中一些预定义变量
>`PROJECT_SOURCE_DIR `工程的根目录
>`PROJECT_BINARY_DIR` 运行cmake命令的目录,通常是${PROJECT_SOURCE_DIR}/build
>`CMAKE_INCLUDE_PATH` 环境变量,非cmake变量
>`CMAKE_LIBRARY_PATH` 环境变量
>`CMAKE_CURRENT_SOURCE_DIR` 当前处理的CMakeLists.txt所在的路径
>`CMAKE_CURRENT_BINARY_DIR` target编译目录
>使用ADD_SURDIRECTORY(src bin)可以更改此变量的值
>SET(EXECUTABLE_OUTPUT_PATH <新路径>)并不会对此变量有影响,只是改变了最终目标文件的存储路径
>CMAKE_CURRENT_LIST_FILE 输出调用这个变量的CMakeLists.txt的完整路径
>CMAKE_CURRENT_LIST_LINE 输出这个变量所在的行
>CMAKE_MODULE_PATH 定义自己的cmake模块所在的路径
>SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake),然后可以用INCLUDE命令来调用自己的模块
>EXECUTABLE_OUTPUT_PATH 重新定义目标二进制可执行文件的存放位置
>LIBRARY_OUTPUT_PATH 重新定义目标链接库文件的存放位置
>PROJECT_NAME 返回通过PROJECT指令定义的项目名称
>CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 用来控制IF ELSE语句的书写方式
>系统信息

## cmake常用命令
基本语法规则：

>* cmake变量使用 ```${}```方式取值,但是在IF控制语句中是直接使用变量名
>* 环境变量使用```$ENV{}```方式取值,使用```SET(ENV{VAR} VALUE)```赋值
>* 指令(参数1 参数2…)参数使用括弧括起,参数之间使用空格或分号分开。
```cmake
# 以ADD_EXECUTABLE指令为例：
ADD_EXECUTABLE(hello main.c func.c)或者
ADD_EXECUTABLE(hello main.c;func.c)
```
>PROJECT
>PROJECT(projectname [CXX] [C] [Java])
>指定工程名称,并可指定工程支持的语言。支持语言列表可忽略,默认支持所有语言

>SET
>SET(VAR [VALUE] [CACHE TYPE DOCSTRING [FORCE]])
>定义变量(可以定义多个VALUE,如SET(SRC_LIST main.c util.c reactor.c))

>MESSAGE
>MESSAGE([SEND_ERROR | STATUS | FATAL_ERROR] “message to display” …)
>向终端输出用户定义的信息或变量的值
>SEND_ERROR, 产生错误,生成过程被跳过
>STATUS, 输出前缀为—的信息
>FATAL_ERROR, 立即终止所有cmake过程

>ADD_EXECUTABLE
>ADD_EXECUTABLE(bin_file_name ${SRC_LIST})
>生成可执行文件

>ADD_LIBRARY
>ADD_LIBRARY(libname [SHARED | STATIC | MODULE] [EXCLUDE_FROM_ALL] SRC_LIST)
>生成动态库或静态库
>SHARED 动态库
>STATIC 静态库
>MODULE 在使用dyld的系统有效,若不支持dyld,等同于SHARED
>EXCLUDE_FROM_ALL 表示该库不会被默认构建

>SET_TARGET_PROPERTIES
>设置输出的名称,设置动态库的版本和API版本

>CMAKE_MINIMUM_REQUIRED
>CMAKE_MINIMUM_REQUIRED(VERSION version_number [FATAL_ERROR])
>声明CMake的版本要求

>ADD_SUBDIRECTORY
>ADD_SUBDIRECTORY(src_dir [binary_dir] [EXCLUDE_FROM_ALL])
>向当前工程添加存放源文件的子目录,并可以指定中间二进制和目标二进制的存放位置
>EXCLUDE_FROM_ALL含义：将这个目录从编译过程中排除

>SUBDIRS
>deprecated,不再推荐使用
>(hello sample)相当于分别写ADD_SUBDIRECTORY(hello),ADD_SUBDIRECTORY(sample)

>INCLUDE_DIRECTORIES
>INCLUDE_DIRECTORIES([AFTER | BEFORE] [SYSTEM] dir1 dir2 … )
>向工程添加多个特定的头文件搜索路径,路径之间用空格分隔,如果路径包含空格,可以使用双引号将它括起来,默认的行为为追加到当前头文件搜索路径的后面。有如下两种方式可以控制搜索路径添加的位置：
>CMAKE_INCLUDE_DIRECTORIES_BEFORE,通过SET这个cmake变量为on,可以将添加的头文件搜索路径放在已有路径的前面
>通过AFTER或BEFORE参数,也可以控制是追加还是置前

>LINK_DIRECTORIES
>LINK_DIRECTORIES(dir1 dir2 …)
>添加非标准的共享库搜索路径

>TARGET_LINK_LIBRARIES
>TARGET_LINK_LIBRARIES(target lib1 lib2 …)
>为target添加需要链接的共享库

>ADD_DEFINITIONS
>向C/C++编译器添加-D定义
>ADD_DEFINITIONS(-DENABLE_DEBUG -DABC),参数之间用空格分隔

>ADD_DEPENDENCIES
>ADD_DEPENDENCIES(target-name depend-target1 depend-target2 …)
>定义target依赖的其他target,确保target在构建之前,其依赖的target已经构建完毕

>AUX_SOURCE_DIRECTORY
>AUX_SOURCE_DIRECTORY(dir VAR)
>将某个目录dir下所有的源代码文件并将列表存储在一个变量VAR中

>EXEC_PROGRAM
>EXEC_PROGRAM(Executable [dir where to run] [ARGS <args>][OUTPUT_VARIABLE <var>] [RETURN_VALUE <value>])
>用于在指定目录运行某个程序（默认为当前CMakeLists.txt所在目录）,通过ARGS添加参数,通过OUTPUT_VARIABLE和RETURN_VALUE获取输出和返回值

## 单目录文单件demo1
**`[main.cpp]`**
```cpp
#include <iostream>
int add(int i,int j){
	return i+j;
}
int main(int argc,char *argv[])
{
	int x=3;
	int y=4;
	std::cout<<"x+y="<<add(x,y)<<std::endl;	
	return 0;
}
```
**`[CMakeLists.txt]`**
```c
CMAKE_MINIMUM_REQUIRED(VERSION 3.10) #版本
PROJECT(demo1) #项目名称
ADD_EXECUTABLE(demo1  main.cpp)
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210128201953322.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)

## 单目录多文件demo2
```[main.cpp]```
```cpp
#include <iostream>
#include "add.h"


int main(int argc,char *argv[])
{
	int x=3;
	int y=2;
	std::cout<<"x+y="<<add(x,y)<<std::endl;
	return 0;
}
```
`[add.h]`
```cpp
#ifndef ADD_H_
#define ADD_H_

int  add(int,int);

#endif
```

`[add.cpp]`
```cpp
#include "add.h"

int add(int i,int j){
    return i+j;
}
```

`[CMakeLists.txt]`
```cpp
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT(demo2)
AUX_SOURCE_DIRECTORY(./ DIR_SRCS)  #将./下的所有文件存在DIR_SRCS

ADD_EXECUTABLE(demo2  ${DIR_SRCS})
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210128204144466.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)

## 多目录多文件demo3
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210128211240580.png)
>需要分别在项目根目录 Demo3 和 math 目录里各编写一个 CMakeLists.txt 文件

根目录下的`CMakeLists.txt`

```c
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT(demo3)

INCLUDE_DIRECTORIES(./mylib) #添加头文件目录

ADD_SUBDIRECTORY(./mylib) #向当前工程添加存放源文件的子目录

AUX_SOURCE_DIRECTORY(./ DIR_SRCS) #将某个目录./下所有文件并将列表存储在一个变量DIR_SRCS中

ADD_EXECUTABLE(demo3  ${DIR_SRCS}) #生成可执行文件

TARGET_LINK_LIBRARIES(demo3 Mylib) #为demmo3添加需要链接的库

```
子目录中的 `CMakeLists.txt`
```c
aux_source_directory(. DIR_LIB_SRCS)

add_library(Mylib STATIC ${DIR_LIB_SRCS}) #生成静态库
#add_library(Mylib SHARED ${DIR_LIB_SRCS}) #生成动态库
```
`【main.cpp】`
```c
#include <iostream>
#include "add.h"
#include "sub.h"

int main(int argc,char *argv[])
{
	int x=5;
	int y=3;
	std::cout<<"x+y="<<add(x,y)<<std::endl;
	std::cout<<"x-y="<<sub(x,y)<<std::endl;

	return 0;
}

```

## 标准工程 demo4
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210128224547418.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
`mylib下的CMakeLists.txt`
```	c

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(../include)

AUX_SOURCE_DIRECTORY(./ DIR_SRC)

ADD_LIBRARY(Mylib SHARED ${DIR_SRC})

```
`src下的CMakeList.txt`
```c

INCLUDE_DIRECTORIES(../include)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

AUX_SOURCE_DIRECTORY(./ DIR_SRC)
ADD_EXECUTABLE(demo4 ${DIR_SRC})

TARGET_LINK_LIBRARIES(demo4 Mylib)
```
`根目录下的CMakeLists.txt`
```c
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT(demo4)

ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/mylib)

ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src)
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210128224121158.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
## cmake构建protobuf
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210129175640475.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
`CMakeLists.txt`
```c
cmake_minimum_required(VERSION 3.10)
project(protobuf_study)

# 查找 protobuf
find_package(Protobuf REQUIRED)
if (PROTOBUF_FOUND)
    message("protobuf found")
else (PROTOBUF_FOUND)
    message(FATAL_ERROR "Cannot find Protobuf")
endif (PROTOBUF_FOUND)

# 生成 proto 为 .cpp 和 .h
set(PROTO_FILES proto/simple.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
message("PROTO_FILES = ${PROTO_FILES}")
message("PROTO_SRCS = ${PROTO_SRCS}")
message("PROTO_HDRS = ${PROTO_HDRS}")


# 关联 protobuf 到最后的二进制文件
add_executable(protobuf_study
        src/main.cpp
        ${PROTO_SRCS}
        ${PROTO_HDRS})

# 设置可执行程序输出路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


target_include_directories(${PROJECT_NAME}
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
        PUBLIC ${PROTOBUF_INCLUDE_DIRS})
message("PROJECT_NAME = ${PROJECT_NAME}")
message("CMAKE_CURRENT_BINARY_DIR = ${CMAKE_CURRENT_BINARY_DIR}")
message("CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
message("PROTOBUF_INCLUDE_DIRS = ${PROTOBUF_INCLUDE_DIRS}")

message("PROTOBUF_LIBRARIES = ${PROTOBUF_LIBRARIES}")
target_link_libraries(protobuf_study ${PROTOBUF_LIBRARIES})

```

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210129175924252.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
## cmake构建ros
![在这里插入图片描述](https://img-blog.csdnimg.cn/20210131180513403.png)

![在这里插入图片描述](https://img-blog.csdnimg.cn/20210131175852699.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzM4OTY2ODY3,size_16,color_FFFFFF,t_70)
`velocity_publish.cpp`
```cpp 
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
	// ROS节点初始化
	ros::init(argc, argv, "velocity_publisher");

	// 创建节点句柄
	ros::NodeHandle n;
	
	ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

	// 设置循环的频率
	ros::Rate loop_rate(3);

	int count = 0;
	while (ros::ok())
	{
	    // 初始化geometry_msgs::Twist类型的消息
		geometry_msgs::Twist vel_msg;
		vel_msg.linear.x = 0.5;
		vel_msg.angular.z = 0.2;

	    // 发布消息
		turtle_vel_pub.publish(vel_msg);
		ROS_INFO("Publsh turtle velocity %0.2f m/s, %0.2f rad/s", 
				vel_msg.linear.x, vel_msg.angular.z);

	    // 按照循环频率延时
	    loop_rate.sleep();
	}

	return 0;
}
```
`velocity_subsciber.cpp`
```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// 接收到订阅的消息后，会进入消息回调函数
void vCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // 将接收到的消息打印出来
    ROS_INFO("Turtle velocity: x:%0.6f, z:%0.6f",msg->linear.x, msg->linear.z);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "velocity_subscriber");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    ros::Subscriber pose_sub = n.subscribe("/turtle1/cmd_vel", 10, vCallback);

    // 循环等待回调函数
    ros::spin();

    return 0;
}

```
`CMakeLists.txt`
```cpp
cmake_minimum_required(VERSION 2.8.3)
project(learning_topic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIB_DIRS})

set(EXECUTABLE_OUTPUT_PATH ./bin)

add_executable(velocity_publisher src/velocity_publisher.cpp)
target_link_libraries(velocity_publisher ${catkin_LIBRARIES})

add_executable(velocity_subscriber src/velocity_subscriber.cpp)
target_link_libraries(velocity_subscriber ${catkin_LIBRARIES})

```