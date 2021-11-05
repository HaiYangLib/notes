# ROS安装
在ubuntu18.04上安装ros\
## 安装准备
```bash
sudo apt-get update
sudo apt-get upgrade
```
## 添加ROS的镜像源
```bash
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
```
## 配置密钥
```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
> 备选地址
> hkp://pgp.mit.edu:80
> hkp://keyserver.ubuntu.com:80 
## 更新源
```bash
sudo apt-get update
```
## 安装melodic
```bash
sudo apt install ros-melodic-desktop-full
```
> 库文件安装在 /opt/ros/melodic
## ROS初始化
```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## 安装工具依赖
```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```
## 初始化rosdep
```bash
sudo rosdep init
rosdep update
```
## 启动ROS
```bash
roscore
```