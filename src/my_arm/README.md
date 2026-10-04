# 六轴机械臂与双指夹爪

ROS 2 Jazzy、Gazebo Harmonic、MoveIt 2。ROS 包名是 `my_arm01`。

## 直接操纵机械臂

只启动一次以下命令，等待 RViz 和“六轴机械臂控制”窗口出现：

```bash
source /opt/ros/jazzy/setup.bash
cd /root/catkin_ws
colcon build --packages-select my_arm01 --symlink-install
source install/setup.bash
ros2 launch my_arm01 control.launch.py
```

控制面板的六个滑块设置关节角度，点击“发送关节角度”执行；夹爪半开度
0–40 mm，对应两指总开口 0–80 mm。“回到初始姿态”会发送六轴初始位置。
首次启动时控制按钮保持灰色，直到 Gazebo、关节状态和控制器都就绪。
若需要 Gazebo 自身的三维窗口，可用
`ros2 launch my_arm01 control.launch.py gazebo_gui:=true`。
启动文件始终先运行 Gazebo 仿真服务器，因此 Gazebo 图形窗口不影响控制器。

控制面板直接发送关节指令，**不检查桌面或方块碰撞**；接近物体时请用
MoveIt 规划命令。每次只运行一个 `control.launch.py`、`moveit.launch.py`
或 `gazebo.launch.py` 实例；不要同时运行 `display.launch.py`，它发布的是
演示用关节状态，会与真实仿真反馈冲突。用 Ctrl+C 关闭当前实例后再切换。

## MoveIt 目标位置与抓取

在已启动 `control.launch.py` 的情况下，另开 Ubuntu 终端：

```bash
source /opt/ros/jazzy/setup.bash
source /root/catkin_ws/install/setup.bash
ros2 run my_arm01 motion_plan.py pose --x 0.35 --y 0 --z 0.43 --ros-args -p use_sim_time:=true
ros2 run my_arm01 motion_plan.py home --ros-args -p use_sim_time:=true
```

坐标单位是米，参考系 `world`，目标点是两指之间的 `grasp_center`；
姿态固定为夹爪朝下。先试算可加 `--plan-only`。命令会把桌面、地面和
方块加入规划场景；无法规划时不会驱动机械臂。

默认桌面与方块的自动抓取测试：

```bash
ros2 run my_arm01 motion_plan.py pick --ros-args -p use_sim_time:=true
```

该命令先规划到方块上方，然后逐步沿直线接近、夹持、抬升、放回。
碰撞场景中的“附着”只用于路径检查，Gazebo 中仍由手指接触实际夹持。
程序检查方块实际抬升至少 60 mm 和释放后的高度。重复抓取前重启仿真
恢复初始场景。普通 `gazebo.launch.py` 不启动 MoveIt，因此不能运行
`motion_plan.py`。

调试控制器：

```bash
ros2 control list_controllers
```

应看到 `joint_state_broadcaster`、`arm_controller`、
`gripper_fingers_controller` 均为 `active`。夹爪由两个手指的 effort
接口对称驱动，`gripper_action.py` 把单个开合命令转换成双指轨迹。
形状、质量、摩擦和力矩增益是演示用估算值，实机适配前要重新标定。
