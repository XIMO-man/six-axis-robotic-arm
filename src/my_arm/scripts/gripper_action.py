#!/usr/bin/env python3
"""Map a single jaw-opening action to two independently controlled fingers.

URDF retains mimic for display/planning; Gazebo receives explicit symmetric
commands and publishes the two measured states. No object attachment is used.
"""
import math
import threading

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class Gripper(Node):
    def __init__(self):
        super().__init__('gripper_action')
        self.positions = {}
        self.busy = False
        self.lock = threading.Lock()
        self.handles = {}
        group = ReentrantCallbackGroup()
        self.client = ActionClient(self, FollowJointTrajectory,
                                   '/gripper_fingers_controller/follow_joint_trajectory',
                                   callback_group=group)
        self.create_subscription(JointState, '/joint_states', self.state, 10,
                                 callback_group=group)
        self.server = ActionServer(self, GripperCommand, '/gripper_controller/gripper_cmd',
                                   self.execute, goal_callback=self.goal,
                                   cancel_callback=self.cancel, callback_group=group)

    def state(self, msg):
        self.positions.update(zip(msg.name, msg.position))

    def goal(self, request):
        q = request.command.position
        with self.lock:
            if self.busy or not math.isfinite(q) or not 0 <= q <= .04:
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def cancel(self, handle):
        downstream = self.handles.get(bytes(handle.goal_id.uuid))
        if downstream is not None:
            downstream.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def execute(self, handle):
        key = bytes(handle.goal_id.uuid)
        result = GripperCommand.Result()
        try:
            if not self.client.wait_for_server(timeout_sec=60):
                handle.abort()
                return result
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ['finger_joint', 'right_finger_joint']
            point = JointTrajectoryPoint()
            q = handle.request.command.position
            point.positions = [q, -q]
            point.velocities = [0., 0.]
            point.time_from_start.sec = 2
            goal.trajectory.points = [point]
            downstream = await self.client.send_goal_async(goal)
            self.handles[key] = downstream
            if not downstream.accepted:
                handle.abort()
                return result
            if handle.is_cancel_requested:
                await downstream.cancel_goal_async()
            response = await downstream.get_result_async()
            left = self.positions.get('finger_joint', float('nan'))
            right = self.positions.get('right_finger_joint', float('nan'))
            result.position = (left - right) / 2
            result.reached_goal = max(abs(left-q), abs(right+q)) < .001
            result.stalled = not result.reached_goal
            result.effort = 0.0  # Position control: no calibrated force estimate.
            if handle.is_cancel_requested:
                handle.canceled()
            elif response.status == 4 and response.result.error_code == 0 and abs(left+right) < .004:
                handle.succeed()
            else:
                handle.abort()
            return result
        finally:
            self.handles.pop(key, None)
            with self.lock:
                self.busy = False


def main():
    rclpy.init()
    node = Gripper()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
