from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    domain_file = "/tmp/pddl/gpsrdomid_clean.pddl"
    problem_file = "/tmp/pddl/gpsrprobid.pddl"

    print(" === SEMI-MONOLITHIC SETUP ===")
    print(" Domain :", domain_file)
    print(" Problem:", problem_file)

    # 1. Domain expert (stand-alone)
    domain_expert = Node(
        package='plansys2_domain_expert',
        executable='domain_expert_node',
        name='domain_expert',
        output='screen',
        parameters=[{'domain_file': domain_file}]
    )

    # 2. plansys2_node untuk problem + planner + executor
    plansys2_node = Node(
        package='plansys2_bringup',
        executable='plansys2_node',
        name='plansys2_node',
        output='screen',
        parameters=[
            {'problem_file': problem_file},
            {'autostart': True}
        ]
    )

    # Delay supaya domain_expert siap dulu
    delayed_plansys2_node = TimerAction(period=5.0, actions=[plansys2_node])

    return LaunchDescription([
        domain_expert,
        delayed_plansys2_node
    ])

