from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Gazebo <-> ROS: laser scan out of the sim, belt speed command into it.
    # '[' = Gazebo -> ROS, ']' = ROS -> Gazebo (same convention as the camera bridge).
    conveyor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='conveyor_bridge',
        arguments=[
            '/conveyor/laser_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/conveyor/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
        ],
        output='screen',
    )

    # laser beam broken -> stop belt -> /conveyor/cube_ready -> (pick_place_node)
    # -> /pick_place/done -> belt resumes
    conveyor_controller = Node(
        package='franka_pick_place',
        executable='conveyor_controller_node',
        output='screen',
    )

    return LaunchDescription([
        conveyor_bridge,
        conveyor_controller,
    ])
