from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def _launch_other(package, launch):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package), "launch"), "/" + launch])
    )

def generate_launch_description():
    slam_toolbox = _launch_other(
        package="slam_toolbox",
        launch="online_async_launch.py"
    )
    nav2 = _launch_other(
        package="nav2_bringup",
        launch="navigation_launch.py"
    )
    sllidar = _launch_other(
        package="sllidar_ros2",
        launch="sllidar_a1_launch.py"
    )
    mc = Node(
        package="motorcontrol",
        executable="mc",
    )
    static_tf = Node(
        package="static_tf",
        executable="static_tf_node"
    )
    return LaunchDescription([
        slam_toolbox,
        nav2,
        sllidar,
        mc,
        static_tf
    ])