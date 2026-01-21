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

	# Map fully qualified names to relative ones so the node's namespace can be prepended.
	# In case of the transforms (tf), currently, there doesn't seem to be a better alternative
	# https://github.com/ros/geometry2/issues/32
	# https://github.com/ros/robot_state_publisher/pull/30
	remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]



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
			name='human_finder_node',
			executable='human_finder',
			namespace=namespace,
			parameters=[{'use_sim_time': use_sim_time}],
			output='screen',
			remappings=remappings,
		),
	])
