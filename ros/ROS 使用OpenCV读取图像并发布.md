* image_publisher.cpp

```cpp
//
// Created by hhy on 2021/9/28.
//
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_pub");

  // Reading an image from the file
  cv::Mat cv_image = cv::imread("/home/hhy/图片/test2.jpeg");
  if (cv_image.empty()) {
    ROS_FATAL("Read the picture failed!");
    return -1;
  } else {
    ROS_INFO_STREAM("Read the picture successful!");
  }

  // Convert OpenCV image to ROS message
  ros::NodeHandle node;
  image_transport::ImageTransport transport(node);
  image_transport::Publisher image_pub;
  image_pub = transport.advertise("ImageTopic", 1);
  ros::Time time = ros::Time::now();

  cv_bridge::CvImage cvi;
  cvi.header.stamp = time;
  cvi.header.frame_id = "test_image";
  cvi.encoding = "bgr8";
  cvi.image = cv_image;

  ros::Rate rate(1.0);
  while (ros::ok()) {
    sensor_msgs::Image im;
    cvi.toImageMsg(im);
    ROS_INFO_STREAM("publish");
    image_pub.publish(im);
    rate.sleep();
  }

  return 0;
}
```

* image_subscriber.cpp
```cpp
//
// Created by hhy on 2021/9/28.
//
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

static const char WINDOW[] = "Image window";

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  ROS_INFO_STREAM("Get Msg");
  try {
    cv::imshow(WINDOW, cv_bridge::toCvShare(msg, "bgr8")->image);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow(WINDOW);
  // 在调用cv::startWindowThread();后，即使没有调用waitKey()函数，图片也依然实时刷新。
  // 大家知道，opencv的imshow()函数调用以后，并不立即刷新显示图片，
  // 而是等到waitKey()后才会刷新图片显示，所以我猜测cv::startWindowThread();
  // 是新开一个线程实时刷新图片显示。
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub =
      it.subscribe("ImageTopic", 1, imageCallback);
  ros::spin();
  cv::destroyWindow(WINDOW);
}
```
* CMakeLists.txt


```clike
cmake_minimum_required(VERSION 3.0.2)
project(test_opencv)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        )

find_package(OpenCV REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_publisher src/image_publisher.cpp)
target_link_libraries(image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(image_subscriber src/image_subscriber.cpp)
target_link_libraries(image_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
```
> 左边点击add
> 选中image
> 在image的topic选项中选
> /ImageTopic

![在这里插入图片描述](https://img-blog.csdnimg.cn/7bf186ce6aaa4cea870b6e281dea706d.png?x-oss-process=image/watermark,type_ZHJvaWRzYW5zZmFsbGJhY2s,shadow_50,text_Q1NETiBAaGFuaHkyNDE2,size_20,color_FFFFFF,t_70,g_se,x_16)