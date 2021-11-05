![在这里插入图片描述](https://img-blog.csdnimg.cn/10c459be8c4447e0aa2260b2c1149690.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_15,color_FFFFFF,t_70,g_se,x_16)

```cpp
    /*
      angle-min：开始扫描的角度
      angle-max：结束扫描的角度
      angle-increment：每次扫描增加的角度
      time-increment： 测量的时间间隔
      scan-time ：扫描的时间间隔
      range-min：测距的最小值
      range-max：测距的最大值
      ranges：转一周的测量数据一共360个
      intensities：与设备有关，强度数组长度360
      */
```


```cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

  unsigned int num_readings = 1000;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];

  int count = 10;
  ros::Rate r(1.0);
  while (n.ok()) {
    // generate some fake data for our laser scan
    for (unsigned int i = 0; i < num_readings; ++i) {
      ranges[i] = count;
      intensities[i] = 10 * i;
    }
    ros::Time scan_time = ros::Time::now();

    // populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "sensor_frame";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 100.0;

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for (unsigned int i = 0; i < num_readings; ++i) {
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }

    ROS_INFO_STREAM("publish");
    scan_pub.publish(scan);
    r.sleep();
  }
}
```

![在这里插入图片描述](https://img-blog.csdnimg.cn/f68592bcdccf4113b9c1799a823c18ed.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)