## 自定义OccupancyGrid
![在这里插入图片描述](https://img-blog.csdnimg.cn/63edde4382b94858829674c0e09c748c.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)


```c++
//
// Created by hhy on 2021/9/28.
//
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#define W 10
#define H 10

int main(int argc, char **argv) {
  // 初始化ROS节点 节点名字
  ros::init(argc, argv, "mappublish");
  // 节点句柄
  ros::NodeHandle nh;
  // 发布消息 话题名字 队列大小
  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("map_publish", 1);
  // 定义地图对象
  nav_msgs::OccupancyGrid myMap;

  // 地图数据
  myMap.info.map_load_time = ros::Time(0);
  myMap.info.resolution = 0.5;
  myMap.info.width = W;
  myMap.info.height = H;
  myMap.info.origin.position.x = 0;
  myMap.info.origin.position.y = 0;
  myMap.info.origin.position.z = 0;
  myMap.info.origin.orientation.x = 0;
  myMap.info.origin.orientation.y = 0;
  myMap.info.origin.orientation.z = 0;
  myMap.info.origin.orientation.w = 0;

  for (std::size_t h = 0; h < H; h++) {
    for (std::size_t w = 0; w < W; w++) {
      int8_t a;
      if (w < W / 2)
        a = 10 * w;
      else if (w == W / 2)
        a = 127;
      else
        a = (w - W / 2) * 10;

      if (h < H / 2)
        myMap.data.push_back(a);
      else
        myMap.data.push_back(0);
    }
  }

  // frame id
  myMap.header.frame_id = "map";

  // 消息发布频率
  ros::Rate loop_rate(1);

  while (ros::ok()) {
    ROS_INFO_STREAM("PUB");
    pub.publish(myMap);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

```
## 保存地图
![在这里插入图片描述](https://img-blog.csdnimg.cn/0b58c3605a384313ac4e813e3f4e83b0.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)
> rosrun map_server map_saver map:=/map_publish -f ./mymap
> map:=/map_publish 指定话题
> -f ./mymap 指定保存的位置和名称

地图的yaml格式中有：
* image：包含占用数据的图像文件的路径; 可以是绝对的，或相对于YAML文件的位置
* resolution：地图的分辨率，米/像素
* origin：地图中左下像素的2-D姿态为（x，y，yaw），偏航为逆时针旋转（yaw = 0表示无旋转）。系统的许多部分目前忽略偏航。
* occupancy_thresh：占据概率大于该阈值的像素被认为完全占用。
* free_thresh：占有概率小于该阈值的像素被认为是完全自由的。
* negate：白/黑自由/占用语义是否应该被反转（阈值的解释不受影响）


## roslaunch启动map_server

```xml
<launch>
	<node pkg="map_server" type="map_server" name="map_server" args="-f PATH_TO_YOUR_FILE/mymap" output="screen">
		<remap from="map" to="/< Map Topic >" /> 		
	</node>
</launch>

```
> PATH_TO_YOUR_FILE填写想要保存地图的路径。
> < Map Topic >填写地图的话题类型，比如用octomap_server生成的栅格地图的话题是/projected_map。