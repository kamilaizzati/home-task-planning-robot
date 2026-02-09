#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# PETA LOKASI → KOORDINAT (SESUAI MAP KAMU)
LOCATION_POSES = {
    "dapur": (0.0, 0.0),
    "meja": (1.5, 0.0),
    "rak": (1.5, 1.2),
    "ruangtamu": (0.0, 1.5),
    "lemari": (-1.0, 1.0),
    "kamar": (-1.5, 0.0),
    "jendela": (1.5, -1.2),
    "koridor": (0.0, -1.5),
    "pintu": (-1.5, -1.2),
}

class MoveExecutor(Node):
    def __init__(self):
        super().__init__("move_executor_node")

        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )

        self.get_logger().info("Menunggu Nav2 action server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 siap.")

    def move_to(self, location):
        if location not in LOCATION_POSES:
            self.get_logger().error(f"Lokasi '{location}' tidak dikenal")
            return

        x, y = LOCATION_POSES[location]

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Robot bergerak ke {location} ({x}, {y})")

        self.nav_client.send_goal_async(goal)

def main():
    rclpy.init()
    node = MoveExecutor()

    while rclpy.ok():
        cmd = input("Tujuan (lokasi): ").strip()
        if cmd in ["exit", "quit"]:
            break
        node.move_to(cmd)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

