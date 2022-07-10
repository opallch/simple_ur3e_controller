"""
This script will launch the two nodes that we have written:
the joint trajectory publisher and the joint state subscriber.
"""
import sys
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def generate_launch_description():

    # read values from our self-defined yml file for defining ROS parameters
    config = PathJoinSubstitution(
            [FindPackageShare("my_controller"), 
            "config", 
            "publisher_config.yaml"]
        )

    # try:
    #     get_package_share_directory("my_controller")
    
    # except PackageNotFoundError:
    #     print(
    #         "ERROR:"
    #         "Could not find package 'my_controller'. "
    #     )
    #     sys.exit(1)
    
    # NOTE: executable is the name of the entry points defined in `setup.py`
    return LaunchDescription(
        [   
            Node(
                package="my_controller",
                executable="joint_controller", 
                name="publisher_joint_trajectory_controller",
                parameters=[config], # tell ROS to set the ROS parameters from our self-defined yml file
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
            Node(
                package="my_controller",
                executable="joint_subscriber", 
                name="subscriber_joint_state",
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
        ]
    )