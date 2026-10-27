from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
from robom1_sim.robot_config import load_robot_config, node_parameters


def launch_setup(context):
    description = Path(get_package_share_directory('robom1_description'))
    config = Path(get_package_share_directory('robom1_bringup')) / 'config' / 'sim.yaml'
    design_path = str(Path(LaunchConfiguration('robot_config').perform(context)).expanduser().resolve())
    design = load_robot_config(design_path)
    base_parameters, scene_parameters = node_parameters(design)
    robot = xacro.process_file(str(description / 'urdf' / 'robom1.urdf.xacro'),
                              mappings={'robot_config': design_path}).toxml()
    demo, follow, rviz = [LaunchConfiguration(key) for key in ['demo', 'follow_goal', 'rviz']]
    impact = LaunchConfiguration('impact_demo')
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot, 'use_sim_time': False}], output='screen'),
        Node(package='robom1_control', executable='base_sim', name='base_sim',
             parameters=[str(config), base_parameters, {'use_sim_time': False}], output='screen'),
        Node(package='robom1_sim', executable='scene_sim', name='scene_sim',
             parameters=[str(config), scene_parameters, {'demo_aim': ParameterValue(demo, value_type=bool),
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
    ]


def generate_launch_description():
    default = Path(get_package_share_directory('robom1_description')) / 'config' / 'robot.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('robot_config', default_value=str(default),
                              description='Robot design YAML; shared by URDF, sensors and collision.'),
        DeclareLaunchArgument('impact_demo', default_value='false', choices=['true', 'false'],
                              description='Stationary-base incoming feed, racket collision and prediction.'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('demo', default_value='true', choices=['true', 'false'],
                              description='Follow and aim at shuttle ground truth (not perception).'),
        DeclareLaunchArgument('follow_goal', default_value='false', choices=['true', 'false'],
                              description='With demo=false, follow odom-frame goal_pose commands.'),
        OpaqueFunction(function=launch_setup),
    ])
