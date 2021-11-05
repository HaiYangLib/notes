![在这里插入图片描述](https://img-blog.csdnimg.cn/a48227898e574d918676780898b25f96.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_15,color_FFFFFF,t_70,g_se,x_16)
> name表示不同的表示方式，可以是像素值，亮度等。
> values则是典型的依赖于传感器的亮度值返回，表示测量的质量，可以是灰度图像的灰度值，也可以是类似laserscan的强度。

```cpp
//
// Created by hhy on 2021/9/28.
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "point_cloud_publisher");

  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("/cloud", 50);

  unsigned int num_points = 20000;

  int count = 0;
  ros::Rate r(1.0);
  while (n.ok()) {
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "sensor_frame";

    cloud.points.resize(num_points);

    // we'll also add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(num_points);

    // generate some fake data for our point cloud
    for (unsigned int i = 0; i < num_points; ++i) {
      cloud.points[i].x = 1 + i / 1000;
      cloud.points[i].y = 1;
      cloud.points[i].z = 1;
      cloud.channels[0].values[i] = 100 * i;
    }

    ROS_INFO_STREAM("publish");
    cloud_pub.publish(cloud);
    // ++count;
    r.sleep();
  }
}
```


![在这里插入图片描述](https://img-blog.csdnimg.cn/ecc4e8f57a284e399ebc3bb7ce691939.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)