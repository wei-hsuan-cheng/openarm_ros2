import os
import launch
import launch_ros
import xacro

from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def generate_launch_description():
    xacro_file = get_package_file(
        "openarm_bimanual_description", "urdf/openarm_bimanual.urdf.xacro"
    )
    urdf = xacro.process_file(xacro_file).toprettyxml(indent="  ")
    default_rviz_config_path = get_package_file(
        "openarm_bimanual_description", "rviz/robot_description.rviz"
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                name="robot_state_publisher",
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": urdf}],
            ),
            launch_ros.actions.Node(
                name="joint_state_publisher_gui",
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig",
                default_value=str(default_rviz_config_path),
                description="Absolute path to rviz config file",
            ),
            launch_ros.actions.Node(
                name="rviz",
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
            ),
        ]
    )
