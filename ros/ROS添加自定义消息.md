# ROS添加自定义消息
* 创建文件夹msg,和scripts，src同级目录
* 创建文件gps.msg，并写入
```c
float32 lng
float32 lat
```
* 在package.xml文件中添加
```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```
* 在CMakeList.txt文件中
找到find_package 添加message_generation
```c
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        serial
        message_generation
        )
```
找到catkin_package 添加CATKIN_DEPENDS message_runtime
```c
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES ros_control_pkg
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
        CATKIN_DEPENDS message_runtime
)
```
找到add_message_files 添加FILES gps.msg
```c
add_message_files(
        FILES
        gps.msg
#   Message1.msg
#   Message2.msg
)
```
找到generate_messages添加 DEPENDENCIES std_msgs
```c
generate_messages(
    DEPENDENCIES
    std_msgs
)
```