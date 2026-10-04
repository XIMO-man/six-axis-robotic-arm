#!/usr/bin/env python3
"""Manual ROS 2 controller panel for the six arm joints and both gripper fingers."""
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from PyQt5 import QtCore, QtWidgets


JOINTS = [f'joint_{i}' for i in range(1, 7)]
LIMITS = [(-math.pi, math.pi), (-math.pi/2, math.pi/2),
          (-5*math.pi/6, 5*math.pi/6), (-math.pi, math.pi),
          (-2*math.pi/3, 2*math.pi/3), (-math.pi, math.pi)]
HOME = [0., -.45, .9, 0., .45, 0.]


class ControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_panel')
        self.positions = {}
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/arm_controller/follow_joint_trajectory')
        self.gripper = ActionClient(self, GripperCommand,
                                    '/gripper_controller/gripper_cmd')

    def on_state(self, msg):
        self.positions.update(zip(msg.name, msg.position))


class Panel(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.synced = False
        self.busy = False
        self.setWindowTitle('六轴机械臂控制')
        self.resize(560, 360)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel('拖动滑块后点击“发送关节角度”；数值单位为度。'))
        self.sliders = []
        for index, (minimum, maximum) in enumerate(LIMITS, 1):
            row = QtWidgets.QHBoxLayout()
            row.addWidget(QtWidgets.QLabel(f'关节 {index}'))
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(round(math.degrees(minimum)), round(math.degrees(maximum)))
            slider.setSingleStep(1)
            value = QtWidgets.QLabel('0°')
            value.setMinimumWidth(54)
            slider.valueChanged.connect(lambda v, label=value: label.setText(f'{v}°'))
            row.addWidget(slider)
            row.addWidget(value)
            layout.addLayout(row)
            self.sliders.append(slider)
        buttons = QtWidgets.QHBoxLayout()
        self.send_button = QtWidgets.QPushButton('发送关节角度')
        self.home_button = QtWidgets.QPushButton('回到初始姿态')
        self.send_button.clicked.connect(self.send_arm)
        self.home_button.clicked.connect(self.send_home)
        buttons.addWidget(self.send_button)
        buttons.addWidget(self.home_button)
        layout.addLayout(buttons)
        grip = QtWidgets.QHBoxLayout()
        grip.addWidget(QtWidgets.QLabel('夹爪半开度'))
        self.grip_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.grip_slider.setRange(0, 40)
        self.grip_label = QtWidgets.QLabel('0 mm')
        self.grip_slider.valueChanged.connect(
            lambda v: self.grip_label.setText(f'{v} mm'))
        self.grip_button = QtWidgets.QPushButton('发送夹爪开度')
        self.grip_button.clicked.connect(self.send_grip)
        grip.addWidget(self.grip_slider)
        grip.addWidget(self.grip_label)
        grip.addWidget(self.grip_button)
        layout.addLayout(grip)
        self.status = QtWidgets.QLabel('等待仿真和控制器…')
        layout.addWidget(self.status)
        layout.addWidget(QtWidgets.QLabel('关节滑块直接控制执行器；障碍物附近请用 MoveIt 规划移动。'))
        for button in (self.send_button, self.home_button, self.grip_button):
            button.setEnabled(False)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(30)

    def refresh(self):
        if not rclpy.ok():
            self.timer.stop()
            QtWidgets.QApplication.quit()
            return
        rclpy.spin_once(self.node, timeout_sec=0)
        have_joints = all(name in self.node.positions for name in JOINTS)
        ready = (have_joints and self.node.arm.server_is_ready()
                 and self.node.gripper.server_is_ready())
        for button in (self.send_button, self.home_button, self.grip_button):
            button.setEnabled(ready and not self.busy)
        if ready and not self.synced:
            for slider, name in zip(self.sliders, JOINTS):
                slider.setValue(round(math.degrees(self.node.positions[name])))
            self.grip_slider.setValue(round(1000*self.node.positions.get('finger_joint', 0.)))
            self.synced = True
            self.status.setText('控制器已连接。')

    def send_home(self):
        for slider, value in zip(self.sliders, HOME):
            slider.setValue(round(math.degrees(value)))
        self.send_arm()

    def send_arm(self):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [math.radians(slider.value()) for slider in self.sliders]
        point.velocities = [0.] * len(JOINTS)
        point.time_from_start.sec = 4
        goal.trajectory.points = [point]
        self.submit(self.node.arm, goal, '机械臂')

    def send_grip(self):
        goal = GripperCommand.Goal()
        goal.command.position = self.grip_slider.value() / 1000
        goal.command.max_effort = 30.
        self.submit(self.node.gripper, goal, '夹爪')

    def submit(self, client, goal, label):
        self.busy = True
        self.status.setText(f'{label}正在运动…')
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda done: self.accepted(done, label))

    def accepted(self, future, label):
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('控制器拒绝了命令')
            handle.get_result_async().add_done_callback(
                lambda done: self.completed(done, label))
        except Exception as exc:
            self.busy = False
            self.status.setText(f'{label}失败：{exc}')

    def completed(self, future, label):
        try:
            response = future.result()
            if response.status != 4:
                raise RuntimeError(f'动作状态 {response.status}')
            inner = response.result
            if hasattr(inner, 'error_code') and inner.error_code != 0:
                raise RuntimeError(f'控制器错误 {inner.error_code}')
            self.status.setText(f'{label}运动完成。')
        except Exception as exc:
            self.status.setText(f'{label}失败：{exc}')
        finally:
            self.busy = False


def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    node = ControlNode()
    window = Panel(node)
    window.show()
    try:
        app.exec_()
    finally:
        window.timer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
