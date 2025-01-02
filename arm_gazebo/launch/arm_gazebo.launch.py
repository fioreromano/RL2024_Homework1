
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Definisci un nodo o una descrizione di launch
    arm_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('arm_gazebo'), 'launch', 'arm_world.launch.py'])
        ])
    )

    arm_control_launch = TimerAction(
        period=5.0,  # Ritardo di 3 secondi
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([FindPackageShare('arm_control'), 'launch', 'arm_control.launch.py'])
                ])
            )
        ]
    )

    return LaunchDescription([
        arm_world_launch,
        arm_control_launch
    ])

