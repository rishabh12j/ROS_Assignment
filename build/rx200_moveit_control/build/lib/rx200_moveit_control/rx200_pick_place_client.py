#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MotionPlanRequest, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Quaternion


class MoveItEEClient(Node):
    def __init__(self, control_group='arm'):
        super().__init__('rx200_moveit_control')
        self._client = ActionClient(self, MoveGroup, '/move_action')
        while not self._client.wait_for_server(1.0):
            self.get_logger().warning('Waiting for MoveGroup action server...')
        
        # Set control group based on argument
        if control_group == 'gripper':
            self.group_name = 'interbotix_gripper'
        else:
            self.group_name = 'interbotix_arm'
        
        self.ee_link = 'rx200/ee_gripper_link'
        self.base_link = 'rx200/base_link'
        self.gripper_joint = 'left_finger'
        
        self.get_logger().info(f'Node initialized for control group: {self.group_name}!')

    def send_gr_pose(self, open=True):
        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.allowed_planning_time = 2.0
        req.num_planning_attempts = 1

        jc = JointConstraint()
        jc.joint_name = self.gripper_joint
        jc.position = 0.05 if open else 0.0
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
        
        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_respose_cb)

    def send_pose(self, x, y, z, w=1.0):
        pose = PoseStamped()
        pose.header.frame_id = self.base_link
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=w)

        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.allowed_planning_time = 5.0
        req.num_planning_attempts = 3

        pc = PositionConstraint()
        pc.header.frame_id = self.base_link
        pc.link_name = self.ee_link
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.01]
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
        
        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_respose_cb)

    def _goal_respose_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt goal rejected')
            return
        self.get_logger().info('MoveIt Goal accepted')
        goal_handle.get_result_async().add_done_callback(self._result_cb)
    
    def _feedback_cb(self, feedback_msg):
        state = getattr(feedback_msg.feedback, "state", "<unknown>")
        self.get_logger().info(f"[Feedback] {state}")
    
    def _result_cb(self, future):
        result = future.result().result
        code = getattr(result.error_code, 'val', )
        self.get_logger().info(f"[Result] error_code {code}")



def main():
    rclpy.init()
    node = MoveItEEClient(control_group='gripper')
    # Example to open the gripper
    node.send_gr_pose(open=True)
    rclpy.spin_once(node)
    rclpy.shutdown()

def main1():
    rclpy.init()
    node = MoveItEEClient(control_group='arm')
    # Example waypoint motion for arm
    x_o, y_o = 0.2, 0.0
    x_t, y_t = 0.4, 0.0
    z_hover = 0.15
    z_pick = 0.1
    waypoints = [
        (x_o, y_o, z_hover),
        (x_t, y_t, z_hover),
        (x_t, y_t, z_pick),
        (x_t, y_t, z_hover)
    ]
    for x, y, z in waypoints:
        node.get_logger().info(f"{x},{y},{z}")
        node.send_pose(x, y, z)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()   # Change to main1() to run gripper control
