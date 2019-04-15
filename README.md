# iqr_pan_tilt
IQR Camera pan_tilt driver

## 环境要求
- Ubuntu 16.04
- GCC 4.5以上
- ROS Kinetic


## 安装与编译
更新软件
```shell
sudo apt-get update
```
创建ROS工作空间
```shell
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```
克隆代码并编译
```shell
git clone https://github.com/I-Quotient-Robotics/iqr_pan_tilt.git
cd ..
catkin_make
```
环境设置
```shell
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 使用
启动驱动节点
```shell
roslaunch pan_tilt_bringup panTilt_view.launch
```
发送位置消息
```shell
rostopic pub /iqr/pan_tilt_cmd pan_tilt_driver/PanTiltCmd "yaw: 30.0 pitch: 30.0 speed: 20"
```
