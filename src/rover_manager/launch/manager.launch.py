from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    	Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["0", "0", "0", "0", "0", "0", "map", "rover/map"]),
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["0", "0", "1.5", "0", "0", "0", "map", "exp00"]),
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["2", "0", "1.5", "0", "0", "0", "map", "exp10"]),
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["4", "0", "1.5", "0", "0", "0", "map", "exp20"]),
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["6", "0", "1.5", "0", "0", "0", "map", "exp30"]),
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["0", "1", "1.5", "0", "0", "0", "map", "exp01"]),
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["2", "1", "1.5", "0", "0", "0", "map", "exp11"]),
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["4", "1", "1.5", "0", "0", "0", "map", "exp21"]),
        Node(package = "tf2_ros", 
             executable = "static_transform_publisher",
             arguments = ["6", "1", "1.5", "0", "0", "0", "map", "exp21"]),
        Node(
            package='rover_manager',
            executable='rover_manager',
            #name='mimic',
            name='rover_manager',
            output="screen"
        ),
        Node(
            package='seed',
            executable='seed',
            name='mimic',
            output="screen",
            #output={"both","rover_ros2_log.txt"},
            arguments=["pdt_rover"],
            parameters=[
               #{"frames_to_explore": ["exp1","exp2","exp3","exp4"]}
               #{"frames_to_explore": ["exp00","exp10","exp20","exp30","exp01","exp11","exp21","exp31"]}
               {"frames_to_explore": ["exp00","exp10","exp01","exp11"]}
            ]
        )
    ])
