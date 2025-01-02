from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
 
def generate_launch_description():
    # Declare arguments
    declared_arguments = []
        
    arm_description_path = os.path.join(get_package_share_directory('arm_description'))
    description_file_xacro = os.path.join(arm_description_path, "urdf", "arm.urdf.xacro")
    
	
    r_d_x = {"robot_description":Command(['xacro ', description_file_xacro])}

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )  

    robot_state_publisher_node_arm = Node(
        package="robot_state_publisher",	 #ros2 run robot_state_publisher robot_state_publisher
        executable="robot_state_publisher",
        output="both",
        parameters=[r_d_x,
                    {"use_sim_time": True},
            ],
        remappings=[('/robot_description', '/robot_description')]
    )

   
    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )
    position = [0.0, 0.0, 0.45]
    
    declared_arguments.append(DeclareLaunchArgument('gz_args', default_value='-r -v 1 empty.sdf',
                              description='Arguments for gz_sim'),) 
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'arm',
                   '-allow_renaming', 'true',
                    "-x", str(position[0]),
                    "-y", str(position[1]),
                    "-z", str(position[2]),],
    )
    ign = [gazebo_ignition, gz_spawn_entity]
    
    
    bridge_camera = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args', 
            '-r', '/camera:=/videocamera',
        ],
        output='screen'
    )
    
    
    nodes_to_start = [
        robot_state_publisher_node_arm,
        *ign,
        bridge_camera
    ]

    
    return LaunchDescription(declared_arguments + nodes_to_start) 

