# six-axis-robotic-arm
六轴机械臂 ROS 项目

## 项目简介
基于ROS的六轴机械臂运动学仿真与控制工程，包含URDF模型、运动学求解、轨迹规划相关代码。

## 环境
- Ubuntu 24.04
- ROS 2 / ROS Noetic
- Catkin工作空间

## 使用方法
```bash
# 编译
colcon build
# 环境变量
source install/setup.bash
# 启动仿真
roslaunch six_axis_robotic_arm six_axis_robotic_arm.launch
```
