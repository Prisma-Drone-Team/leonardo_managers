from launch import LaunchDescription
from launch_ros.actions import Node

# ALL STANDARD TFs: ["expl1","expr1","expcl","expcen","expcr","expal","exptwin","expar","explft","expsqr","exprgt"]

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='seed_pdt',
            executable='seed',
            name='mimic',
            output="screen",
            #output={"both","drone_ros2_log.txt"},
            arguments=["pdt_drone"],
            parameters=[
               #{"frames_to_explore": ["expl1","expcl","expcen","expcr","expal","exptwin","expar","expsqr"]} #Initial (ERF1)
               #{"frames_to_explore": ["exp00","exp01","exp10","exp11","expal","expar"]}, # (ERF2)
               {"frames_to_explore": ["exp00","exp11","exp10","expal"]}, # (ERF3)
               {"path": ["exp00","exp10","exp11","expal"]}
            ]
        ),
        Node(
            package='seed_pdt',
            executable='seed',
            name='mimic',
            output="screen",
            #output={"both","rover_ros2_log.txt"},
            arguments=["pdt_rover"],
            parameters=[
               #{"frames_to_explore": ["expl1","expr1","expcl","expcen","expcr","expal","exptwin","expar","explft","expsqr"]} #Initial (ERF1)
               #{"frames_to_explore": ["expl1","expr1","expcl","expcen","expcr","expal","exptwin","expar","explft","expsqr"]}, # (ERF2)
               {"frames_to_explore": ["expr1","expcen","expcr","expl2","expr2","expal","exptwin","expar","explft","expsqr"]}, # (ERF3)
               {"path": ["expr1","expcen","expcr","expr2","expl2","expal","explft","expsqr","expar","exptwin"]}
            ]
        )
    ])