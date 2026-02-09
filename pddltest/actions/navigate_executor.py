#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from plansys2_executor.action_executor_client import ActionExecutorClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

from tf_transformations import quaternion_from_euler


class NavigateExecutor(Node):

    def __init__(self):
        super().__init__('navigate_executor')

        # REGISTER KE PLANSYS
        self.executor = ActionExecutorClient(
            self,
            'pindah',   # HARUS SAMA PERSIS DENGAN NAMA ACTION DI DOMAIN
            self.do_work
        )

        # NAV2 ACTION CLIENT
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info("NavigateExecutor ready")

    def do_work(self, args):

        # args = ['dapur', 'rak']
        from_loc = args[0]
        to_loc   = args[1]

        self.get_logger().info(f"Executing pindah {from_loc} -> {to_loc}")

        # HARD-CODE LOKASI (AMAN BUAT DEMO)
        pose_map = {
            'dapur': (0.0, 0.0, 0.0),
            'meja': (1.0, 0.0, 0.0),
            'rak': (2.0, 0.0, 0.0),
            'ruangtamu': (0.0, 1.5, 0.0),
            'lemari': (2.0, 1.5, 0.0),
        }

        if to_loc not in pose_map:
            self.executor.finish(False, 1.0, f"Unknown location {to_loc}")
            return

        x, y, yaw = pose_map[to_loc]

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal = NavigateToPose.Goal()
        goal.pose = pose

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.executor.finish(False, 1.0, "Nav2 not available")
            return

        send_goal = self.nav_client.send_goal_async(goal)
        send_goal.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.executor.finish(False, 1.0, "Goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.executor.finish(True, 1.0, "Navigation success")


def main():
    rclpy.init()
    node = NavigateExecutor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

