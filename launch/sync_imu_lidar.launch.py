from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    imu_ahrs_launch = os.path.join(
        get_package_share_directory('imu_ahrs'),
        'launch',
        'imu_ahrs.launch.py'
    )
    psd360_launch = os.path.join(
        get_package_share_directory('psd360'),
        'launch',
        'psd360.launch.py'
    )

    velodyne_launch = os.path.join(
        get_package_share_directory('velodyne'),
        'launch',
        'velodyne-all-nodes-VLP16-launch.py'
    )

    return LaunchDescription([
        Node(
            package='sync_imu_lidar',
            executable='sync_imu_lidar',
            name='sync_imu_lidar',
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(velodyne_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_ahrs_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(psd360_launch)
        ),

    ])