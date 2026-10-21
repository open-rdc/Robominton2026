from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


def generate_launch_description():
    description = Path(get_package_share_directory('robom1_description'))
    config = Path(get_package_share_directory('robom1_bringup')) / 'config' / 'sim.yaml'
    parameters = yaml.safe_load(config.read_text())
    geometry = parameters['base_sim']['ros__parameters']
    mappings = {key: str(geometry[key]) for key in ['wheelbase', 'track', 'wheel_radius']}
    dimensions = parameters['scene_sim']['ros__parameters']
    mappings.update({key: str(value) for key, value in dimensions.items()
                     if key.startswith('racket_') and key != 'racket_restitution'})
    robot = xacro.process_file(str(description / 'urdf' / 'robom1.urdf.xacro'),
                              mappings=mappings).toxml()
    demo, follow, rviz = [LaunchConfiguration(key) for key in ['demo', 'follow_goal', 'rviz']]
    impact = LaunchConfiguration('impact_demo')
    return LaunchDescription([
        DeclareLaunchArgument('impact_demo', default_value='false', choices=['true', 'false'],
                              description='Stationary-base incoming feed, racket collision and prediction.'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('demo', default_value='true', choices=['true', 'false'],
                              description='Follow and aim at shuttle ground truth (not perception).'),
        DeclareLaunchArgument('follow_goal', default_value='false', choices=['true', 'false'],
                              description='With demo=false, follow odom-frame goal_pose commands.'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': False}], output='screen'),
        Node(package='robom1_control', executable='base_sim', name='base_sim',
             parameters=[str(config), {'use_sim_time': False}], output='screen'),
        Node(package='robom1_sim', executable='scene_sim', name='scene_sim',
             parameters=[str(config), {'demo_aim': ParameterValue(demo, value_type=bool),
                                       'demo_swing': ParameterValue(demo, value_type=bool),
                                       'impact_demo': ParameterValue(impact, value_type=bool),
                                       'use_sim_time': False}], output='screen'),
        Node(package='robom1_control', executable='goal_follower', name='goal_follower',
             condition=IfCondition(PythonExpression(["'", impact, "' == 'false' and ('", demo, "' == 'true' or '", follow, "' == 'true')"])),
             parameters=[str(config), {'demo': ParameterValue(demo, value_type=bool),
                                       'use_sim_time': False}], output='screen'),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(rviz),
             arguments=['-d', str(description / 'rviz' / 'sim.rviz')],
             parameters=[{'use_sim_time': False}], output='screen'),
    ])
