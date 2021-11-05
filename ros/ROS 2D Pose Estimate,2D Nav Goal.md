![在这里插入图片描述](https://img-blog.csdnimg.cn/74461d1f224e4b308860329410f192da.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)

```cpp
//
// Created by hhy on 2021/9/29.
//

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

// PoseWithCovarianceStamped
void GetPoseEstimate(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial) {
  float x = initial->pose.pose.position.x;
  float y = initial->pose.pose.position.y;
  float z = initial->pose.pose.position.z;
  ROS_INFO_STREAM("GetPoseEstimate   x:" << x << " y:" << y << " z:" << z);
}

void GetNavGoal(const geometry_msgs::PoseStamped::ConstPtr &end) {
  float x = end->pose.position.x;
  float y = end->pose.position.y;
  float z = end->pose.position.z;
  ROS_INFO_STREAM("GetNavGoal   x:" << x << " y:" << y << " z:" << z);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sub_point");
  ros::NodeHandle nh;
  auto sub_start = nh.subscribe("/initialpose", 1, GetPoseEstimate);
  auto sub_goal = nh.subscribe("/move_base_simple/goal", 1,GetNavGoal);
  ROS_INFO_STREAM("spin");
  ros::spin();
}
```