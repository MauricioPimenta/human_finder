from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

	namespace = LaunchConfiguration('namespace')
	use_sim_time = LaunchConfiguration('use_sim_time')


	return LaunchDescription([

		DeclareLaunchArgument(
			'namespace',
			default_value='a200_0000',
			description='Robot Namespace'
		),

		DeclareLaunchArgument(
			'use_sim_time',
			default_value='true',
			description='Use simulation time if true'
		),



		Node(
			package='human_finder',
			executable='human_finder_node',
			namespace=namespace,
			parameters=[{'use_sim_time': use_sim_time}]
		),
	])
