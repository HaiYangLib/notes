![在这里插入图片描述](https://img-blog.csdnimg.cn/01df0733a63d4df5910d3af4af0fc0be.png)


```c
uint8 ARROW=0//箭头
uint8 CUBE=1//立方体
uint8 SPHERE=2//球
uint8 CYLINDER=3//圆柱体
uint8 LINE_STRIP=4//线条（点的连线）
uint8 LINE_LIST=5//线条序列
uint8 CUBE_LIST=6//立方体序列
uint8 SPHERE_LIST=7//球序列
uint8 POINTS=8//点集
uint8 TEXT_VIEW_FACING=9//显示3D的文字
uint8 MESH_RESOURCE=10//网格？
uint8 TRIANGLE_LIST=11//三角形序列
//对标记的操作
uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header
string ns   //命名空间namespace，就是你理解的那样
int32 id    //与命名空间联合起来，形成唯一的id，这个唯一的id可以将各个标志物区	分开来，使得程序可以对指定的标志物进行操作
int32 type  //类型
int32 action 	//操作，是添加还是修改还是删除
geometry_msgs/Pose pose       # Pose of the object
geometry_msgs/Vector3 scale   # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color      # Color [0.0-1.0]
duration lifetime             # How long the object should last before being 	automatically deleted.  0 means forever
bool frame_locked             # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points//这个是在序列、点集中才会用到，指明序列中每个点的位置
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/652a6076f36048d49973ca09f3435159.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)

```cpp
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Publisher marker_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  uint32_t shape = visualization_msgs::Marker::CUBE;

  ros::Rate r(1);
  int id = 0;
  int p=0;
  while (ros::ok()) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = id++;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    p++;
    marker.pose.position.x = p;
    marker.pose.position.y = p;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(5);

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
        return 0;
      }
      ROS_WARN("Please create a subscriber to the marker");
      sleep(1);
    }
    ROS_INFO_STREAM("PUB");
    marker_pub.publish(marker);

    // Cycle between different shapes
    switch (shape) {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }

    r.sleep();
  }
}
```