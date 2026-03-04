#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Quaternion

import tf2_ros
from tf2_ros import TransformException

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


def yaw_to_quat(yaw: float) -> Quaternion:
    """Create a quaternion from a yaw angle (Z axis)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class HumanFinder(Node):
    def __init__(self):
        super().__init__('human_finder_node')

        # ---- Parameters (edit defaults if you want) ----
        self.declare_parameter('namespace', 'a200_0000')
        self.declare_parameter('fixed_frame', 'map')           # frame you want the goal in
        self.declare_parameter('human_frame', 'detected_human')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('side_offset_m', 0.8)           # how far to stand to the side
        self.declare_parameter('tf_timeout_s', 0.2)
        self.declare_parameter('pause_topic', 'pause')         # relative -> /a200_0000/pause when namespaced

        self.ns = self.get_parameter('namespace').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.human_frame = self.get_parameter('human_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.side_offset = float(self.get_parameter('side_offset_m').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout_s').value)
        self.pause_topic = self.get_parameter('pause_topic').value

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Pause exploration ----
        self.pause_pub = self.create_publisher(Bool, self.pause_topic, 10)

        # ---- Nav2 action client ----
        # Use the fully qualified action name with namespace
        self.nav_action_name = f'/{self.ns}/navigate_to_pose'
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)

        self.sent_goal = False

        # Tick until we succeed once
        self.timer = self.create_timer(0.5, self.tick)

        self.get_logger().info(f"Waiting for TF {self.fixed_frame} -> {self.human_frame}")
        self.get_logger().info(f"Will pause exploration by publishing Bool on '{self.pause_topic}' "
                               f"(in namespace this becomes '/{self.ns}/{self.pause_topic}')")
        self.get_logger().info(f"Will send Nav2 goal to action: {self.nav_action_name}")

    def tick(self):
        if self.sent_goal:
            return

        # Let TF listener process callbacks
        # rclpy.spin_once(self, timeout_sec=0.05)

        # 1) Try to get human pose in fixed_frame
        try:
            tf_human = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.human_frame,
                Time(),  # latest available
                timeout=Duration(seconds=self.tf_timeout)
            )
        except TransformException as ex:
            self.get_logger().debug(f"TF not ready yet: {ex}")
            return

        # 2) Also get robot pose to compute "side" direction
        try:
            tf_robot = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout)
            )
        except TransformException as ex:
            self.get_logger().warn(f"Got human TF but not robot TF yet: {ex}")
            return

        hx = tf_human.transform.translation.x
        hy = tf_human.transform.translation.y
        rx = tf_robot.transform.translation.x
        ry = tf_robot.transform.translation.y

        # Vector from robot -> human
        dx = hx - rx
        dy = hy - ry
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            self.get_logger().warn("Robot and human positions are almost the same; cannot compute side offset.")
            return

        dx /= norm
        dy /= norm

        # Side direction: rotate (dx,dy) +90deg => (-dy, dx)
        sx = -dy
        sy = dx

        goal_x = hx + sx * self.side_offset
        goal_y = hy + sy * self.side_offset

        # Face the human from the goal pose
        yaw = math.atan2(hy - goal_y, hx - goal_x)

        # 3) Pause exploration (publish once)
        pause_msg = Bool()
        pause_msg.data = True
        self.pause_pub.publish(pause_msg)
        self.get_logger().warn("Published pause=True to stop exploration.")

        # 4) Send Nav2 goal
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"Nav2 action server not available: {self.nav_action_name}")
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.fixed_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(goal_x)
        goal.pose.pose.position.y = float(goal_y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = yaw_to_quat(yaw)

        self.get_logger().warn(
            f"Sending goal beside human: ({goal_x:.2f}, {goal_y:.2f}), yaw={yaw:.2f} rad, frame='{self.fixed_frame}'"
        )

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.on_goal_response)

        self.sent_goal = True
        # Stop ticking; we’ve done our job
        self.timer.cancel()

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("NavigateToPose goal was rejected.")
            return

        self.get_logger().info("NavigateToPose goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)

    def on_nav_result(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Navigation finished. status={status}, result={result}")

        # Optional: keep node alive, or exit cleanly.
        # If you want to exit when done:
        rclpy.shutdown()


def main():
    rclpy.init()
    node = HumanFinder()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()