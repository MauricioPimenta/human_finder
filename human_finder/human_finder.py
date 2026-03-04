#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from action_msgs.msg import GoalStatus

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


def quat_to_yaw(q: Quaternion) -> float:
    """Extract planar yaw from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class HumanFinder(Node):
    def __init__(self):
        super().__init__('human_finder_node')

        # ---- Parameters (edit defaults if you want) ----
        self.declare_parameter('namespace', 'a200_0000')
        self.declare_parameter('fixed_frame', 'map')           # frame you want the goal in
        self.declare_parameter('human_frame', 'detected_human')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('side_offset_m', 0.5)           # how far to stand to the side
        self.declare_parameter('tf_timeout_s', 0.2)
        self.declare_parameter('pause_topic', 'pause')         # relative -> /a200_0000/pause when namespaced
        self.declare_parameter('pause_settle_s', 1.0)          # wait for explore_lite to cancel its goal
        self.declare_parameter('max_goal_retries', 3)

        self.ns = self.get_parameter('namespace').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.human_frame = self.get_parameter('human_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.side_offset = float(self.get_parameter('side_offset_m').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout_s').value)
        self.pause_topic = self.get_parameter('pause_topic').value
        self.pause_settle_s = float(self.get_parameter('pause_settle_s').value)
        self.max_goal_retries = int(self.get_parameter('max_goal_retries').value)

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Pause exploration ----
        self.pause_pub = self.create_publisher(Bool, self.pause_topic, 10)

        # ---- Nav2 action client ----
        # Use the fully qualified action name with namespace
        self.nav_action_name = f'/{self.ns}/navigate_to_pose'
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action_name)

        self.goal_in_flight = False
        self.pause_sent = False
        self.pause_sent_at = None
        self.current_goal = None
        self.goal_attempts = 0

        # Tick until we succeed once
        self.timer = self.create_timer(0.5, self.tick)

        self.get_logger().info(f"Waiting for TF {self.fixed_frame} -> {self.human_frame}")
        self.get_logger().info(f"Will pause exploration by publishing Bool on '{self.pause_topic}' "
                               f"(in namespace this becomes '/{self.ns}/{self.pause_topic}')")
        self.get_logger().info(f"Will send Nav2 goal to action: {self.nav_action_name}")

    def tick(self):
        if self.goal_in_flight:
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
            # print the transform for debugging
            self.get_logger().info(f"Got human TF: translation=\n({tf_human.transform.translation.x:.2f}, "
                                   f"{tf_human.transform.translation.y:.2f}, "
                                   f"{tf_human.transform.translation.z:.2f}), "
                                   f"rotation=({tf_human.transform.rotation.x:.2f}, "
                                   f"{tf_human.transform.rotation.y:.2f}, "
                                   f"{tf_human.transform.rotation.z:.2f}, "
                                   f"{tf_human.transform.rotation.w:.2f})")
        except TransformException as ex:
            self.get_logger().debug(f"TF not ready yet: {ex}")
            return

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
        human_yaw = quat_to_yaw(tf_human.transform.rotation)
        rx = tf_robot.transform.translation.x
        ry = tf_robot.transform.translation.y

        # Perpendicular directions to the human heading. Either side is acceptable.
        side_a_x = math.sin(human_yaw)
        side_a_y = -math.cos(human_yaw)
        side_b_x = -side_a_x
        side_b_y = -side_a_y

        candidate_a = (hx + side_a_x * self.side_offset, hy + side_a_y * self.side_offset)
        candidate_b = (hx + side_b_x * self.side_offset, hy + side_b_y * self.side_offset)

        distance_to_a = math.hypot(candidate_a[0] - rx, candidate_a[1] - ry)
        distance_to_b = math.hypot(candidate_b[0] - rx, candidate_b[1] - ry)

        if distance_to_a <= distance_to_b:
            goal_x, goal_y = candidate_a
            chosen_side = "A"
        else:
            goal_x, goal_y = candidate_b
            chosen_side = "B"

        yaw = human_yaw

        # 3) Pause exploration (publish once)
        if not self.pause_sent:
            pause_msg = Bool()
            pause_msg.data = True
            self.pause_pub.publish(pause_msg)
            self.pause_sent = True
            self.pause_sent_at = self.get_clock().now()
            self.get_logger().warn("Published pause=True to stop exploration.")
            return

        elapsed = (self.get_clock().now() - self.pause_sent_at).nanoseconds / 1e9
        if elapsed < self.pause_settle_s:
            self.get_logger().info(
                f"Waiting {self.pause_settle_s:.2f}s for exploration to stop before sending Nav2 goal "
                f"({elapsed:.2f}s elapsed)."
            )
            return

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
            f"Sending goal on human side {chosen_side}: ({goal_x:.2f}, {goal_y:.2f}), "
            f"yaw={yaw:.2f} rad, frame='{self.fixed_frame}'"
        )

        self.current_goal = goal
        self.goal_attempts += 1
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self.on_goal_response)

        self.goal_in_flight = True

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("NavigateToPose goal was rejected.")
            self.goal_in_flight = False
            return

        self.get_logger().info("NavigateToPose goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)

    def on_nav_result(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Navigation finished. status={status}, result={result}")
        self.goal_in_flight = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Reached human approach goal successfully.")
            self.timer.cancel()
            rclpy.shutdown()
            return

        if status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            if self.goal_attempts < self.max_goal_retries:
                self.get_logger().warn(
                    f"Navigation did not complete (status={status}). Retrying "
                    f"{self.goal_attempts}/{self.max_goal_retries} after keeping exploration paused."
                )
                self.pause_sent_at = self.get_clock().now()
                return

            self.get_logger().error(
                f"Navigation failed after {self.goal_attempts} attempts with status={status}."
            )
            self.timer.cancel()
            rclpy.shutdown()
            return

        self.get_logger().warn(f"Navigation ended with unexpected status={status}.")
        self.timer.cancel()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = HumanFinder()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()
