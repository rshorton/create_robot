from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='create_driver',
            executable='create_driver'
        ),
		# Launch the robot state, publishes urdf model
        IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('create_description'), 'launch', 'create_2.launch.py'))
        )
        ,
        # Launch rplidar.  This will take care of setting the serial port device and
        # specify 'laser' as the frame for scans.
        IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
               os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar.launch.py'))
        )
    ])
