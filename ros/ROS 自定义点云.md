```cpp
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_point");

  ros::NodeHandle nh;
  ros::Publisher pcl_pub =
      nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;

  // Fill in the cloud data
  cloud.width = 8;
  cloud.height = 1; //此处也可以为cloud.width = 4; cloud.height = 2;
  cloud.points.resize(cloud.width * cloud.height);

  cloud.points[0].x = 1;
  cloud.points[0].y = 1;
  cloud.points[0].z = 0;

  cloud.points[1].x = -1;
  cloud.points[1].y = 1;
  cloud.points[1].z = 0;

  cloud.points[2].x = 1;
  cloud.points[2].y = -1;
  cloud.points[2].z = 0;

  cloud.points[3].x = -1;
  cloud.points[3].y = -1;
  cloud.points[3].z = 0;

  cloud.points[4].x = 1;
  cloud.points[4].y = 1;
  cloud.points[4].z = 2;

  cloud.points[5].x = -1;
  cloud.points[5].y = 1;
  cloud.points[5].z = 2;

  cloud.points[6].x = 1;
  cloud.points[6].y = -1;
  cloud.points[6].z = 2;

  cloud.points[7].x = -1;
  cloud.points[7].y = -1;
  cloud.points[7].z = 2;
  // Convert the cloud to ROS message
  pcl::toROSMsg(cloud, output);
  output.header.frame_id = "odom";

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    ROS_INFO("publish");
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

```
* CmakeLists.txt
```clike
cmake_minimum_required(VERSION 3.0.2)
project(test_pcl)

find_package(catkin REQUIRED COMPONENTS
        pcl_ros
        roscpp
        rospy
        sensor_msgs
        std_msgs
        )

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(test_point src/test_point.cpp)
target_link_libraries(test_point ${catkin_LIBRARIES} ${PCL_LIBRARIES})
```
* package.xml

```xml
<build_depend>pcl_ros</build_depend> 
<build_export_depend>pcl_ros</build_export_depend>
<exec_depend>pcl_ros</exec_depend>
```

> 打开rviz
> 在rviz中增加PointCloud2d
> topic 选 /pcl_output
> fixed Frame 输入odom

![在这里插入图片描述](https://img-blog.csdnimg.cn/5bb536b5da1c4555b3fa99a0b6b3b3e9.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_17,color_FFFFFF,t_70,g_se,x_16)