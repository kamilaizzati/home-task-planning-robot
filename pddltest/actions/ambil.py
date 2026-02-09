#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from plansys2_msgs.msg import ActionExecutionInfo
import time

class AmbilAction(Node):
    def __init__(self):
        super().__init__("ambil_action")

        self.create_subscription(
            ActionExecutionInfo,
            "/plansys2/executor/action_execution",
            self.callback,
            10
        )

        self.get_logger().info("Node Ambil siap.")

    def callback(self, msg: ActionExecutionInfo):
        if msg.action == "ambil" and msg.status == 1:
            obj = msg.arguments[0]
            loc = msg.arguments[1]

            self.get_logger().info(f"Mengambil {obj} di {loc} ...")
            time.sleep(2)
            self.get_logger().info("Objek berhasil diambil.")

def main():
    rclpy.init()
    node = AmbilAction()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
