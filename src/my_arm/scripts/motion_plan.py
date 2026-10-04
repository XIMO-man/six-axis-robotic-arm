#!/usr/bin/env python3
"""MoveIt pose commands and a collision-checked pick/replace demonstration."""
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, Constraints, JointConstraint,
    OrientationConstraint, PositionConstraint, PlanningScene,
    PlanningSceneComponents, AllowedCollisionEntry,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene
from shape_msgs.msg import SolidPrimitive

from grasp_demo import Demo


def pose_at(x, y, z):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    # Tool +Z down, finger opening direction along world X.
    pose.orientation.y = 1.0
    pose.orientation.w = 0.0
    return pose


def box(name, dimensions, pose, frame='world'):
    obj = CollisionObject()
    obj.header.frame_id = frame
    obj.id = name
    obj.operation = CollisionObject.ADD
    primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(dimensions))
    obj.primitives = [primitive]
    obj.primitive_poses = [pose]
    return obj


class Planner(Demo):
    def __init__(self):
        super().__init__()
        self.group = ActionClient(self, MoveGroup, '/move_action')
        self.execute_path = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.get_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')

    def ready(self):
        if not self.group.wait_for_server(timeout_sec=60):
            raise RuntimeError('MoveIt unavailable; launch moveit.launch.py first')
        deadline = time.monotonic() + 30
        while self.pose is None or len(self.joints) < 8 or self.get_clock().now().nanoseconds == 0:
            if time.monotonic() > deadline:
                raise RuntimeError('No simulation clock, cube pose, or joint feedback')
            rclpy.spin_once(self, timeout_sec=.1)
        self.settle(.5)

    def service(self, client, request):
        if not client.wait_for_service(timeout_sec=30):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30)
        if not future.done():
            raise RuntimeError(f'Service timed out: {client.srv_name}')
        return future.result()

    def apply(self, scene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        result = self.service(self.scene_client, ApplyPlanningScene.Request(scene=scene))
        if not result.success:
            raise RuntimeError('Planning scene update failed')

    def setup_scene(self):
        cube = self.fresh_cube()
        scene = PlanningScene()
        scene.world.collision_objects = [
            box('table', (.5, .5, .30), pose_at(.45, 0, .15)),
            box('floor', (10., 10., .05), pose_at(0, 0, -.025)),
            box('grasp_cube', (.04, .04, .04), pose_at(cube.x, cube.y, cube.z)),
        ]
        # The base bottom is on the ground plane; no blanket robot collision exemptions.
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        matrix = self.service(self.get_scene_client, request).scene.allowed_collision_matrix
        for name in ['floor', 'base_link']:
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        floor = matrix.entry_names.index('floor')
        base = matrix.entry_names.index('base_link')
        matrix.entry_values[floor].enabled[base] = True
        matrix.entry_values[base].enabled[floor] = True
        scene.allowed_collision_matrix = matrix
        self.apply(scene)
        self.get_logger().info('Planning scene ready: table, floor, cube')

    def plan(self, constraints, plan_only=False):
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.pipeline_id = 'ompl'
        goal.request.planner_id = 'RRTConnectkConfigDefault'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 8.0
        goal.request.max_velocity_scaling_factor = .2
        goal.request.max_acceleration_scaling_factor = .2
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = plan_only
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        result = self.execute(self.group, goal)
        if result.error_code.val != 1:
            raise RuntimeError(f'MoveIt failed: {result.error_code.val}')
        self.get_logger().info('Plan verified' if plan_only else 'Planned motion executed')

    def target(self, x, y, z, plan_only=False):
        target = pose_at(x, y, z)
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = 'world'
        position.link_name = 'grasp_center'
        position.weight = 1.0
        position.constraint_region.primitives = [SolidPrimitive(
            type=SolidPrimitive.SPHERE, dimensions=[.002])]
        position.constraint_region.primitive_poses = [target]
        orientation = OrientationConstraint()
        orientation.header.frame_id = 'world'
        orientation.link_name = 'grasp_center'
        orientation.orientation = target.orientation
        orientation.absolute_x_axis_tolerance = .01
        orientation.absolute_y_axis_tolerance = .01
        orientation.absolute_z_axis_tolerance = .01
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        self.plan(constraints, plan_only)

    def home(self, plan_only=False):
        constraints = Constraints()
        constraints.joint_constraints = [
            JointConstraint(joint_name=f'joint_{i}', position=value,
                            tolerance_above=.005, tolerance_below=.005, weight=1.0)
            for i, value in enumerate([0., -.45, .9, 0., .45, 0.], 1)]
        self.plan(constraints, plan_only)

    def straight(self, x, y, z):
        request = GetCartesianPath.Request()
        request.header.frame_id = 'world'
        request.group_name = 'arm'
        request.link_name = 'grasp_center'
        request.start_state.is_diff = True
        request.waypoints = [pose_at(x, y, z)]
        request.max_step = .005
        request.revolute_jump_threshold = .15
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = .15
        request.max_acceleration_scaling_factor = .15
        request.cartesian_speed_limited_link = 'grasp_center'
        request.max_cartesian_speed = .025
        result = self.service(self.cartesian_client, request)
        if result.error_code.val != 1 or result.fraction < .999:
            raise RuntimeError(f'Cartesian path incomplete ({result.fraction:.1%}); not executed')
        if not self.execute_path.wait_for_server(timeout_sec=10):
            raise RuntimeError('Trajectory execution unavailable')
        executed = self.execute(self.execute_path, ExecuteTrajectory.Goal(trajectory=result.solution))
        if executed.error_code.val != 1:
            raise RuntimeError(f'Trajectory execution failed: {executed.error_code.val}')

    def attach_for_planning(self):
        # Attaching by ID transfers the existing world object into robot state.
        # This only informs collision checking; Gazebo still relies on real contact.
        attached = AttachedCollisionObject()
        attached.link_name = 'grasp_center'
        attached.touch_links = ['left_finger', 'right_finger']
        attached.object.id = 'grasp_cube'
        attached.object.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [attached]
        self.apply(scene)

    def detach_for_planning(self):
        scene = PlanningScene()
        attached = AttachedCollisionObject()
        attached.link_name = 'grasp_center'
        attached.object.id = 'grasp_cube'
        attached.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects = [attached]
        self.apply(scene)
        self.setup_scene()

    def pick(self):
        cube = self.fresh_cube()
        if abs(cube.x-.35) > .015 or abs(cube.y) > .015 or abs(cube.z-.32) > .015:
            raise RuntimeError('Demo expects the initial upright cube; restart simulation')
        if not self.gripper.wait_for_server(timeout_sec=30):
            raise RuntimeError('Gripper controller unavailable')
        x, y, z = cube.x, cube.y, cube.z
        self.grip(.04)
        self.target(x, y, z+.11)
        self.straight(x, y, z+.01)
        self.grip(.018)
        self.settle(1)
        self.attach_for_planning()
        self.straight(x, y, z+.11)
        self.settle(2)
        actual = self.fresh_cube()
        lift = actual.z-z
        self.get_logger().info(f'Actual cube lift: {lift:.4f} m')
        if lift < .06 or math.hypot(actual.x-x, actual.y-y) > .03:
            raise RuntimeError('Physical grasp failed; stop and restart simulation')
        # Keep a 2 mm gap above the table to avoid a numerical contact collision.
        self.straight(x, y, z+.012)
        self.grip(.04)
        self.settle(1)
        self.detach_for_planning()
        self.straight(x, y, z+.11)
        self.settle(1)
        if abs(self.fresh_cube().z-z) > .015:
            raise RuntimeError('Cube release failed')
        self.get_logger().info('PASS: MoveIt planning, Cartesian approach, physical grasp and release')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['scene', 'pose', 'home', 'pick'])
    parser.add_argument('--x', type=float, default=.35)
    parser.add_argument('--y', type=float, default=0.)
    parser.add_argument('--z', type=float, default=.43)
    parser.add_argument('--plan-only', action='store_true')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not all(math.isfinite(v) for v in (args.x, args.y, args.z)):
        parser.error('Coordinates must be finite')
    if args.plan_only and args.command in ('scene', 'pick'):
        parser.error('--plan-only applies only to pose and home')
    rclpy.init()
    node = Planner()
    try:
        node.ready()
        node.setup_scene()
        if args.command == 'pose':
            node.target(args.x, args.y, args.z, args.plan_only)
        elif args.command == 'home':
            node.home(args.plan_only)
        elif args.command == 'pick':
            node.pick()
    except (RuntimeError, ValueError) as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
