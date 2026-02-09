#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from plansys2_msgs.msg import ActionExecutionInfo
import time

class LetakAction(Node):
    def __init__(self):
        super().__init__("letak_action")

        self.create_subscription(
            ActionExecutionInfo,
            "/plansys2/executor/action_execution",
            self.callback,
            10
        )

        self.get_logger().info("Node Letak siap.")

    def callback(self, msg: ActionExecutionInfo):
        if msg.action == "letak" and msg.status == 1:
            obj = msg.arguments[0]
            loc = msg.arguments[1]

            self.get_logger().info(f"Meletakkan {obj} ke {loc}...")
            time.sleep(2)
            self.get_logger().info("Objek sudah diletakkan.")

def main():
    rclpy.init()
    node = LetakAction()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
