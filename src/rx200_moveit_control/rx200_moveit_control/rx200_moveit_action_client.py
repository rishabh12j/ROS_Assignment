#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MotionPlanRequest, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion
import math
from tf_transformations import quaternion_from_euler
import argparse


class MoveItEEClient(Node):
    def __init__(self, control_group='arm'):
        super().__init__('rx200_moveit_control')
        self.motion_done = True
        self.gr_motion_done = True
        self._client = ActionClient(self, MoveGroup, '/move_action')

        while not self._client.wait_for_server(1.0):
            self.get_logger().warning('Waiting for MoveGroup action server')

        if control_group == 'gripper':
            self.group_name = 'interbotix_gripper'
        else:
            self.group_name = 'interbotix_arm'

        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/base_link'
        self.gripper_joint = 'left_finger'

    def send_gr_pose(self, open=True):
        self.gr_motion_done = False
        self.get_logger().info(f'[Control group] {self.group_name} - {"Open" if open else "Close"}')

        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.05 if open else 0.01
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.joint_constraints = [jc]
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.look_around = False

        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def send_pose(self, x, y, z, roll=0.0, pitch=0.0):
        self.motion_done = False
        self.get_logger().info(f'[Control group] {self.group_name}')

        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        yaw = math.atan2(y, x)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 3

        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.ee_link
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.05]
        pc.constraint_region.primitives = [sp]
        pc.constraint_region.primitive_poses = [pose.pose]

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_link
        oc.link_name = self.ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints = [pc]
        goal_constraints.orientation_constraints = [oc]
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.look_around = False

        future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("[Goal] Goal rejected")
            self.motion_done = True
            self.gr_motion_done = True
            return

        self.get_logger().info("[Goal] Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        code = getattr(result.error_code, 'val', -1)
        self.get_logger().info(f"[Result] error_code {code}")
        self.motion_done = True
        self.gr_motion_done = True

    def _feedback_cb(self, feedback_msg):
        state = getattr(feedback_msg.feedback, "state", "<unknown>")
        self.get_logger().info(f"[Feedback] {state}")


def main():
    rclpy.init()
    node = MoveItEEClient(control_group='arm')
    node_g = MoveItEEClient(control_group='gripper')

    parser = argparse.ArgumentParser(description="Pick and Place parameters")
    parser.add_argument('--pick_x', type=float, default=0.2, help='Pick X coordinate')
    parser.add_argument('--pick_y', type=float, default=0.10, help='Pick Y coordinate')
    parser.add_argument('--place_x', type=float, default=-0.4, help='Place X coordinate')
    parser.add_argument('--place_y', type=float, default=-0.20, help='Place Y coordinate')
    parser.add_argument('--z_hover', type=float, default=0.25, help='Z hover height')
    parser.add_argument('--z_pick', type=float, default=0.15, help='Z pick height')
    args = parser.parse_args()

    x_o = args.pick_x
    y_o = args.pick_y
    x_t = args.place_x
    y_t = args.place_y
    z_hover = args.z_hover
    z_pick = args.z_pick

    waypoints = [
        (x_o, y_o, z_hover, 'open'),
        (x_o, y_o, z_pick, 'close'),
        (x_o, y_o, z_hover, None),
        (x_t, y_t, z_hover, None),
        (x_t, y_t, z_pick, 'open'),
        (x_t, y_t, z_hover, 'close'),
    ]

    for x, y, z, gripper_action in waypoints:

        while not node.motion_done or not node_g.gr_motion_done:
            rclpy.spin_once(node, timeout_sec=0.1)
            rclpy.spin_once(node_g, timeout_sec=0.1)

        node.get_logger().info(f"[Moving] ({x},{y},{z})")
        node.send_pose(x, y, z)

        while not node.motion_done:
            rclpy.spin_once(node, timeout_sec=0.1)

        if gripper_action == 'open':
            node_g.send_gr_pose(open=True)
            while not node_g.gr_motion_done:
                rclpy.spin_once(node_g, timeout_sec=0.1)

        elif gripper_action == 'close':
            node_g.send_gr_pose(open=False)
            while not node_g.gr_motion_done:
                rclpy.spin_once(node_g, timeout_sec=0.1)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

# To run this script, first launch the MoveIt2 node for the RX200 robot with a fake hardware interface:

#ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=rx200 hardware_type:=fake
#ros2 run rx200_moveit_control rx200_moveit_client --pick_x 0.3 --pick_y 0.1 --place_x -0.5 --place_y -0.2