import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(pkg, path):
    full = os.path.join(get_package_share_directory(pkg), path)
    with open(full, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    moveit_pkg = get_package_share_directory('franka_fr3_moveit_config')
    desc_pkg   = get_package_share_directory('franka_description')

    # robot_description — MUST match the Gazebo URDF exactly
    urdf_xacro = os.path.join(desc_pkg, 'robots', 'fr3', 'fr3.urdf.xacro')
    robot_description = {'robot_description': ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_xacro,
                 ' ros2_control:=false hand:=true robot_type:=fr3',
                 ' arm_prefix:= robot_ip:=dont-care',
                 ' use_fake_hardware:=false fake_sensor_commands:=false']),
        value_type=str)}

    srdf_xacro = os.path.join(desc_pkg, 'robots', 'fr3', 'fr3.srdf.xacro')
    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', srdf_xacro,
                 ' hand:=true arm_prefix:=']),
        value_type=str)}

    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')

    ompl_pipeline = {'move_group': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters':
            'default_planner_request_adapters/AddTimeOptimalParameterization '
            'default_planner_request_adapters/ResolveConstraintFrames '
            'default_planner_request_adapters/FixWorkspaceBounds '
            'default_planner_request_adapters/FixStartStateBounds '
            'default_planner_request_adapters/FixStartStateCollision '
            'default_planner_request_adapters/FixStartStatePathConstraints',
        'start_state_max_bounds_error': 0.1,
    }}
    ompl_pipeline['move_group'].update(
        load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml'))

    moveit_controllers = {
        'moveit_simple_controller_manager':
            load_yaml('franka_fr3_moveit_config', 'config/fr3_controllers.yaml'),
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_pipeline,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', os.path.join(moveit_pkg, 'rviz', 'moveit.rviz')],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_pipeline,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([move_group_node, rviz_node])