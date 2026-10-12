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
    desc_pkg = get_package_share_directory('franka_description')

    # Same PLANNING_GROUP name as pick_place_node.py -- must match, since
    # plan_request_params below is keyed by planning group.
    PLANNING_GROUP = 'fr3_arm'

    urdf_xacro = os.path.join(desc_pkg, 'robots', 'fr3', 'fr3.urdf.xacro')
    srdf_xacro = os.path.join(desc_pkg, 'robots', 'fr3', 'fr3.srdf.xacro')

    robot_description = {'robot_description': ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_xacro,
                 ' ros2_control:=false hand:=true robot_type:=fr3',
                 ' arm_prefix:= robot_ip:=dont-care',
                 ' use_fake_hardware:=false fake_sensor_commands:=false']),
        value_type=str)}

    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', srdf_xacro,
                 ' hand:=true arm_prefix:=']),
        value_type=str)}

    kinematics_yaml = {'robot_description_kinematics':
                        load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')}

    joint_limits_yaml = {'robot_description_planning':
                          load_yaml('franka_pick_place', 'config/fr3_moveit_joint_limits.yaml')}

    planning_pipelines_config = {
        'planning_pipelines': {
            'pipeline_names': ['ompl'],
        },
        'default_planning_pipeline': 'ompl',
    }

    ompl_pipeline = {'ompl': {
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
    ompl_pipeline['ompl'].update(
        load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml'))

    print("=== DEBUG: ompl_pipeline['move_group'] has 'planning_plugin'?",
          'planning_plugin' in ompl_pipeline['ompl'], "===")
    print("=== DEBUG: ompl_planning.yaml raw content:",
          load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml'), "===")

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

    # PlanningComponent's constructor calls plan_request_parameters_.load(node_)
    # with NO namespace argument (verified against moveit2's planning_component.cpp),
    # so it reads from a flat top-level "plan_request_params.*" -- NOT nested under
    # the planning group name. Without this, it defaults to planning_pipeline: ''
    # and fails with "No planning pipeline available for name ''".
    plan_request_params = {
        'plan_request_params': {
            'planning_pipeline': 'ompl',
            'planning_attempts': 10,
            'planning_time': 5.0,
            'max_velocity_scaling_factor': 1.0,
            'max_acceleration_scaling_factor': 1.0,
        }
    }

    print("=== DEBUG: plan_request_params dict itself:", plan_request_params, "===")

    config_dict = {}
    for params in (robot_description, robot_description_semantic, kinematics_yaml,
                   planning_pipelines_config, ompl_pipeline, trajectory_execution,
                   moveit_controllers, planning_scene_monitor_parameters, plan_request_params, joint_limits_yaml,
                   {'use_sim_time': True}):
        config_dict.update(params)

    print("=== DEBUG: config_dict has 'plan_request_params'?",
          'plan_request_params' in config_dict, "===")
    print("=== DEBUG: config_dict has 'robot_description_kinematics'?",
          'robot_description_kinematics' in config_dict, "===")
    print("=== DEBUG: kinematics_yaml raw content:", kinematics_yaml, "===")

    pick_place_node = Node(
        package='franka_pick_place',
        executable='pick_place_node',
        output='screen',
        parameters=[config_dict],
    )

    return LaunchDescription([pick_place_node])