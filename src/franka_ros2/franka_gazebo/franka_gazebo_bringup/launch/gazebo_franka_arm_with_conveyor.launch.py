import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    load_gripper_name = 'load_gripper'
    load_gripper = LaunchConfiguration(load_gripper_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name,
        default_value='true',
        description='true/false for activating the gripper')

    # Reuse the existing arm + Gazebo + controller launch file exactly as-is,
    # forwarding load_gripper through to it. 'controller' is explicitly set to
    # fr3_arm_controller (a JointTrajectoryController) -- the file's own
    # default is 'gravity_compensation_example_controller', which doesn't
    # expose the FollowJointTrajectory action MoveIt's trajectory execution
    # needs, so the arm never actually moved despite planning succeeding.
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_gazebo_bringup'),
                'launch',
                'gazebo_franka_arm_example_controller.launch.py',
            ])
        ),
        launch_arguments={
            load_gripper_name: load_gripper,
            'controller': 'fr3_arm_controller',
            # empty.sdf + the Sensors system, which is what renders camera images
            'gz_args': os.path.join(
                get_package_share_directory('franka_gazebo_bringup'),
                'worlds', 'pick_place_world.sdf') + ' -r',
        }.items(),
    )

    share = get_package_share_directory('franka_gazebo_bringup')
    conveyor_model_path = os.path.join(share, 'models', 'conveyor_belt', 'model.sdf')
    table_model_path = os.path.join(share, 'models', 'table', 'model.sdf')
    cube_model_path = os.path.join(share, 'models', 'cube', 'model.sdf')

    # Layout (world frame == fr3_link0, arm base at the origin). The belt top is at
    # z = 0.495 m; its surface slides 0.3 m toward -X for ~2 s and then stops, so with the
    # model at x = 1.22 it ends up spanning x = 0.42 .. 2.02 m. The near edge has to stay
    # clear of the hand in the arm's ready pose (TCP at x = 0.31, z = 0.49), and the pick
    # point has to stay within top-down reach (x <= ~0.55). Keep these numbers in sync with
    # PICK_POSE / PLACE_POSE and the belt collision box in franka_pick_place/pick_place_node.py.
    spawn_conveyor = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', conveyor_model_path,
            '-name', 'conveyor_belt',
            '-x', '1.22', '-y', '0.0', '-z', '0.0',
        ],
        output='screen',
    )

    # Table beside the arm (top surface at z = 0.40 m).
    spawn_table = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', table_model_path,
            '-name', 'table',
            '-x', '0.35', '-y', '0.55', '-z', '0.0',
        ],
        output='screen',
    )

    # 6 cm cube (centre = model origin) dropped 1 cm onto the belt at its stopped
    # position. Delayed so the belt has finished sliding: at t = 0 the belt does not yet
    # cover x = 0.50.
    spawn_cube = TimerAction(
        period=12.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', cube_model_path,
                '-name', 'cube',
                '-x', '1.80', '-y', '0.0', '-z', '0.535',
            ],
            output='screen',
        )],
    )

    # The pick_place node needs a GripperCommand action server; the arm launch only
    # spawns joint_state_broadcaster + fr3_arm_controller, so load fr3_gripper here.
    # The spawner waits for /controller_manager to come up.
    spawn_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'fr3_gripper',
            '--controller-manager', '/controller_manager',
            '--param-file',
            os.path.join(share, 'config', 'franka_gazebo_controllers.yaml'),
        ],
        output='screen',
    )

    # ---- Overhead camera (fixed, looking straight down at the belt) ------------------------
    # The downward orientation is the sensor pose inside models/overhead_camera/model.sdf
    # (roll -pi/2, pitch +pi/2); CAM_RPY below must match it.
    CAM_XYZ = ('0.9', '0.0', '1.6')
    CAM_RPY = ('-1.5707963', '1.5707963', '0.0')

    spawn_camera = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(share, 'models', 'overhead_camera', 'model.sdf'),
            '-name', 'overhead_camera',
            '-x', CAM_XYZ[0], '-y', CAM_XYZ[1], '-z', CAM_XYZ[2],
        ],
        output='screen',
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/overhead_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image'],
        output='screen',
    )

    # world -> overhead_camera_link (x fwd, y left, z up) -> optical frame (z fwd, x right, y down)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', CAM_XYZ[0], '--y', CAM_XYZ[1], '--z', CAM_XYZ[2],
                   '--roll', CAM_RPY[0], '--pitch', CAM_RPY[1], '--yaw', CAM_RPY[2],
                   '--frame-id', 'world', '--child-frame-id', 'overhead_camera_link'],
        output='screen',
    )
    camera_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
                   '--frame-id', 'overhead_camera_link',
                   '--child-frame-id', 'overhead_camera_optical_frame'],
        output='screen',
    )

    return LaunchDescription([
        load_gripper_launch_argument,
        arm_launch,
        spawn_conveyor,
        spawn_table,
        spawn_cube,
        spawn_gripper_controller,
        spawn_camera,
        camera_bridge,
        camera_tf,
        camera_optical_tf,
    ])