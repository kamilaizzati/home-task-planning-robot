#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from plansys2_msgs.srv import GetPlan
from plansys2_msgs.action import ExecutePlan

class RunPlan(Node):
    def __init__(self):
        super().__init__("run_plan")

        self.cli_plan = self.create_client(GetPlan, "/planner/get_plan")
        self.exec_client = ActionClient(self, ExecutePlan, "/execute_plan")

        self.cli_plan.wait_for_service()
        self.exec_client.wait_for_server()

    def run(self):
        # 1. ambil plan
        req = GetPlan.Request()
        future = self.cli_plan.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if not res.success or not res.plan.items:
            self.get_logger().error("Plan kosong / tidak ditemukan")
            return

        self.get_logger().info("Plan ditemukan, mengeksekusi...")

        # 2. kirim ke executor
        goal = ExecutePlan.Goal()
        goal.plan.items = res.plan.items
        self.exec_client.send_goal_async(goal)

def main():
    rclpy.init()
    node = RunPlan()
    node.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

