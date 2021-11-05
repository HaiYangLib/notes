```cpp
//
// Created by hhy on 2021/9/28.
//
/**
 * 该例程设置/读取海龟例程中的参数
 */
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <string>

int main(int argc, char **argv) {
  int red, green, blue;
  // ROS节点初始化
  ros::init(argc, argv, "parameter_config");
  // 创建节点句柄
  ros::NodeHandle node;

  // 读取背景颜色参数
  ros::param::get("/background_r", red);
  ros::param::get("/background_g", green);
  ros::param::get("/background_b", blue);

  ROS_INFO("Get Backgroud Color[%d, %d, %d]", red, green, blue);

  // 设置背景颜色参数
  ros::param::set("/background_r", 255);
  ros::param::set("/background_g", 255);
  ros::param::set("/background_b", 255);

  // 读取背景颜色参数
  ros::param::get("/background_r", red);
  ros::param::get("/background_g", green);
  ros::param::get("/background_b", blue);

  ROS_INFO("Re-get Backgroud Color[%d, %d, %d]", red, green, blue);

  sleep(1);

  return 0;
}

```
![在这里插入图片描述](https://img-blog.csdnimg.cn/7b637f08d63a4795945fe58e6c16e6ea.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_15,color_FFFFFF,t_70,g_se,x_16)
* test_parameter.launch 

```xml
<launch>
    <node pkg="test_parameter" type="test_parameter" name="test_parameter" output="screen" >
        <rosparam file="$(find test_parameter)/params/params.yaml" command="load" />
        <rosparam file="$(find test_parameter)/params/params.yaml" command="load" ns="test123" />
    </node>
</launch>

```

* params.yaml
```yaml
background_b: 255
background_g: 86
background_r: 69
rosdistro: 'melodic'
roslaunch:
  uris: {host_hcx_vpc__43763: 'http://hcx-vpc:43763/'}
rosversion: '1.14.3'
run_id: 077058de-a38b-11e9-818b-000c29d22e4d
```
![在这里插入图片描述](https://img-blog.csdnimg.cn/3ecfe0e6277c4846b58e4694a488bd47.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_17,color_FFFFFF,t_70,g_se,x_16)