#!/usr/bin/env python3
"""Contact-only pick, hold, replace test for the supplied fixed scene.

No attachment or pose teleportation is used. This is a known-scene trajectory,
not collision-aware motion planning. Restart the simulation before each run.
"""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


def downward_pose(x, z):
    # Shoulder z=.26; elbow-to-wrist pitch axis=.25+.08;
    # wrist pitch axis to grasp center=.08+.06+.04+.06=.24.
    a, b = 0.30, 0.33
    dz = z + 0.24 - 0.26
    cosine = (x*x + dz*dz - a*a - b*b) / (2*a*b)
    if not -1.0 <= cosine <= 1.0:
        raise ValueError('Target is outside the workspace')
    elbow = math.acos(cosine)
    shoulder = math.atan2(x, dz) - math.atan2(b*math.sin(elbow), a+b*math.cos(elbow))
    wrist = math.pi - shoulder - elbow
    if abs(shoulder) > math.pi/2 or abs(elbow) > 5*math.pi/6 or abs(wrist) > 2*math.pi/3:
        raise ValueError('Target exceeds joint limits')
    return [0.0, shoulder, elbow, 0.0, wrist, 0.0]


class Demo(Node):
    def __init__(self):
        super().__init__('grasp_demo')
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.pose = None
        self.pose_received = 0.0
        self.joints = {}
        self.create_subscription(PoseStamped, '/cube_pose', self.on_pose, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)

    def on_pose(self, msg):
        self.pose = msg.pose.position
        self.pose_received = time.monotonic()

    def on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def settle(self, seconds):
        start = self.get_clock().now().nanoseconds
        deadline = time.monotonic() + max(15.0, seconds * 10)
        while (self.get_clock().now().nanoseconds - start) / 1e9 < seconds:
            if time.monotonic() > deadline:
                raise RuntimeError('Simulation clock stopped or too slow')
            rclpy.spin_once(self, timeout_sec=0.1)

    def fresh_cube(self):
        if self.pose is None or time.monotonic() - self.pose_received > 2:
            raise RuntimeError('No fresh cube pose; check Gazebo and bridge')
        return self.pose

    def execute(self, client, goal):
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        if not future.done() or not future.result().accepted:
            raise RuntimeError('Controller rejected goal or timed out')
        handle = future.result()
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=90)
        if not result.done():
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=3)
            raise RuntimeError('Motion timed out; cancellation requested')
        if result.result().status != 4:
            raise RuntimeError(f'Action failed: {result.result()}')
        return result.result().result

    def move(self, z, seconds=4):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f'joint_{i}' for i in range(1, 7)]
        point = JointTrajectoryPoint()
        point.positions = downward_pose(0.35, z)
        point.velocities = [0.0] * 6
        point.time_from_start.sec = seconds
        goal.trajectory.points = [point]
        result = self.execute(self.arm, goal)
        if result.error_code != 0:
            raise RuntimeError(result.error_string)

    def grip(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 30.0
        self.execute(self.gripper, goal)

    def run(self):
        for client in (self.arm, self.gripper):
            if not client.wait_for_server(timeout_sec=60):
                raise RuntimeError('Controller unavailable; launch gazebo.launch.py first')
        deadline = time.monotonic() + 15
        while self.pose is None or not self.joints or self.get_clock().now().nanoseconds == 0:
            if time.monotonic() > deadline:
                raise RuntimeError('Timed out waiting for clock, cube pose, and joint feedback')
            rclpy.spin_once(self, timeout_sec=0.1)
        self.settle(1)
        start = self.fresh_cube()
        if abs(start.x-.35) > .015 or abs(start.y) > .015 or abs(start.z-.32) > .015:
            raise RuntimeError('Cube moved from start position; restart simulation')
        original_z = start.z
        self.get_logger().info('Opening and approaching')
        self.grip(.04)
        self.move(.43, 6)
        self.move(.33, 4)
        self.get_logger().info('Closing onto cube')
        self.grip(.018)
        self.settle(1)
        self.get_logger().info('Lifting and checking actual cube height')
        self.move(.43, 5)
        self.settle(2)
        height = self.fresh_cube().z - original_z
        self.get_logger().info(f'Measured cube lift: {height:.4f} m; fingers: {self.joints.get("finger_joint")}, {self.joints.get("right_finger_joint")}')
        if height < .06:
            raise RuntimeError('Grasp failed: cube did not remain lifted by at least 60 mm')
        self.move(.33, 5)
        self.grip(.04)
        self.settle(1)
        self.move(.43, 4)
        self.settle(1)
        if abs(self.fresh_cube().z - original_z) > .015:
            raise RuntimeError('Release failed: cube did not return to tabletop')
        self.get_logger().info('PASS: contact grasp, lift, hold, and release verified')


def main():
    rclpy.init()
    node = Demo()
    try:
        node.run()
    except (RuntimeError, ValueError) as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
