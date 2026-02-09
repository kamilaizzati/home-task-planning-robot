#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from plansys2_msgs.srv import AddProblem, GetPlan, GetDomain, GetProblem

import yaml
import re
import time


# ========= CONFIG =========
ENTITY_FILE = "/mnt/fd/plansys2_ws/src/service_actions/config/entities.yaml"

BELIEF = {}  # memory object → location


# ========= NLP SUPPORT =========
STOPWORDS = {
    "dong", "nih", "donglah", "ya", "sih",
    "kan", "lah", "ga", "ngga", "please", "tolong"
}

INTENT_MAP = {
    "ambilin": "ambil",
    "mengambil": "ambil",
    "ngambil": "ambil",
    "carikan": "cari",
    "cariin": "cari",
    "taruh": "letak",
    "naruh": "letak",
    "naro": "letak",
    "pindahin": "letak",
    "pindahkan": "letak"
}

QUESTION_HINTS = ["ada", "dimana", "mana", "apa", "ga ya", "ga sih", "ga?"]


def clean_text(text):
    t = text.lower()
    t = re.sub(r"[^\w\s]", " ", t)
    t = " ".join(w for w in t.split() if w not in STOPWORDS)
    for k, v in INTENT_MAP.items():
        t = t.replace(k, v)
    return re.sub(r"\s+", " ", t).strip()


def detect_move_pattern(text, obj):
    m = re.search(rf"{obj}.*dari (\w+) ke (\w+)", text)
    return (m.group(1), m.group(2)) if m else (None, None)


def extract_entities(text):
    with open(ENTITY_FILE) as f:
        d = yaml.safe_load(f)

    objects = [o for o in d.get("objects", []) if o in text]
    locations = [l for l in d.get("locations", []) if l in text]

    return objects[0] if objects else None, locations[0] if locations else None


def detect_intent(tokens):
    if not tokens:
        return None

    first = tokens[0]

    if first in ["ambil", "cari", "letak", "pindah"]:
        return first

    if any(q in tokens for q in QUESTION_HINTS):
        return None  # query → treat as ask/search

    return INTENT_MAP.get(first, None)


# ========= PDDL TEMPLATE =========
PROBLEM_TEMPLATE = """(define (problem auto_problem)
  (:domain home-service)

  (:objects
    gelas piring botol apel baju - object
    dapur meja rak ruangtamu lemari kamar - location
  )

  (:init
    (robot_at dapur)

    (object_at gelas meja)
    (object_at apel meja)
    (object_at botol rak)
    (object_at piring lemari)
    (object_at baju ruangtamu)

    (object_known gelas)
    (object_known apel)
    (object_known botol)
    (object_known piring)
    (object_known baju)

    (gripper_free)

    (path_found dapur meja)
    (path_found dapur rak)
    (path_found dapur ruangtamu)
    (path_found dapur lemari)
    (path_found dapur kamar)

    (path_found meja dapur)
    (path_found rak dapur)
    (path_found ruangtamu dapur)
    (path_found lemari dapur)
    (path_found kamar dapur)
  )

  (:goal
    {goal}
  )
)
"""


def build_goal(intent, obj, loc):
    if intent == "ambil" and obj:
        return f"(and (holding {obj}))"

    if intent == "cari" and obj:
        return f"(and (object_visible {obj}))"

    if intent == "letak" and obj and loc:
        return f"(and (object_at {obj} {loc}))"

    if intent == "pindah" and loc:
        return f"(and (robot_at {loc}))"

    return None


# ========= ROS2 NODE =========
class PlanSysExec(Node):
    def __init__(self):
        super().__init__("plansys_exec")

        self.cli_add_problem = self.create_client(
            AddProblem, "/problem_expert/add_problem"
        )
        self.cli_get_plan = self.create_client(
            GetPlan, "/planner/get_plan"
        )
        self.cli_get_domain = self.create_client(
            GetDomain, "/domain_expert/get_domain"
        )
        self.cli_get_problem = self.create_client(
            GetProblem, "/problem_expert/get_problem"
        )

        for cli, name in [
            (self.cli_add_problem, "add_problem"),
            (self.cli_get_plan, "get_plan"),
            (self.cli_get_domain, "get_domain"),
            (self.cli_get_problem, "get_problem"),
        ]:
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f"Menunggu {name}...")

        print("\nPlanSys2 terhubung.\n")

    def send_problem(self, pddl):
        req = AddProblem.Request()
        req.problem = pddl
        f = self.cli_add_problem.call_async(req)
        rclpy.spin_until_future_complete(self, f)
        return f.result()

    def request_plan(self):
        d = self.cli_get_domain.call_async(GetDomain.Request())
        p = self.cli_get_problem.call_async(GetProblem.Request())

        rclpy.spin_until_future_complete(self, d)
        rclpy.spin_until_future_complete(self, p)

        req = GetPlan.Request()
        req.domain = d.result().domain
        req.problem = p.result().problem

        f = self.cli_get_plan.call_async(req)
        rclpy.spin_until_future_complete(self, f)
        return f.result()


# ========= MAIN LOOP =========
def main():
    rclpy.init()
    node = PlanSysExec()

    print("Ketik perintah robot (exit untuk berhenti)\n")

    while True:
        cmd = input("Perintah: ").strip()
        if cmd.lower() in ["exit", "quit"]:
            break
        if not cmd:
            continue

        txt = clean_text(cmd)
        tokens = txt.split()

        obj, loc = extract_entities(txt)
        intent = detect_intent(tokens)

        print(f"intent_rule: {intent}")
        print(f"object: {obj}")
        print(f"location: {loc}")

        # detect complex 'move object' phrasing
        origin, target = detect_move_pattern(txt, obj) if obj else (None, None)
        if origin and target:
            loc = target
            print(
                f"Dipahami sebagai perintah memindahkan '{obj}' "
                f"dari {origin} ke {target}"
            )

        # Query → treat as "cari"
        if intent is None and obj:
            intent = "cari"
            print("Instruksi diinterpretasikan sebagai perintah mencari.")

        if (
            not intent
            or (intent in ["ambil", "cari", "letak"] and not obj)
            or (intent in ["letak", "pindah"] and not loc)
        ):
            print(
                "Perintah tidak lengkap. "
                "Perlu objek/lokasi/aksi lebih jelas.\n"
            )
            continue

        # Build and send problem
        goal = build_goal(intent, obj, loc)
        problem_str = PROBLEM_TEMPLATE.format(goal=goal)

        res = node.send_problem(problem_str)
        if not res.success:
            print(f"Problem error: {res.error_info}\n")
            continue

        plan = node.request_plan()
        if not plan.success:
            print("Tidak ada rencana.")
            print("➡ Cek apakah objek/lokasi cocok dengan domain.\n")
            continue

        print("\nPlan ditemukan:")
        for i, step in enumerate(plan.plan.items):
            print(f"{i}. {step.action}")


        # Update memory ONLY after successful execution
        if intent == "letak" and obj and loc:
            BELIEF[obj] = loc
            print(
                f"\nBelief diperbarui: "
                f"'{obj}' sekarang di '{loc}'.\n"
            )

        elif intent == "cari" and obj and loc:
            BELIEF[obj] = loc
            print(
                f"\nInfo disimpan: "
                f"'{obj}' kemungkinan ada di '{loc}'.\n"
            )

        print("--------------------------------\n")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

