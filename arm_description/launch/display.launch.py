from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Lista per gli argomenti dichiarati
    declared_arguments = []

    # Dichiarazione dell'argomento rviz_config_file con valore di default
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file", 	# Nome dell'argomento
            default_value=PathJoinSubstitution(
                [FindPackageShare("arm_description"), "config", "rviz", "robot_model_config.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching RViz.",
        )
    )

  
    arm_description_path = get_package_share_directory('arm_description')

    arm_description_urdf = os.path.join(arm_description_path, "urdf", "arm.urdf")
    arm_description_urdf_xacro = os.path.join(arm_description_path, "urdf", "arm.urdf.xacro")

   
    # with open(arm_description_urdf, 'r') as infp:
    #    link_desc = infp.read()

    # robot_description_arm = {"robot_description": link_desc}

    
    r_d_x = {"robot_description": Command(['xacro ', arm_description_urdf_xacro])}

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    robot_state_publisher_node_arm = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher",
        output="both",
        parameters=[r_d_x, 
                    {"use_sim_time": True}, 
                   ],
        remappings=[('/robot_description', '/robot_description')] 
    )

    # Nodo per RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config_file")],
    )

 
    nodes_to_start = [
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

