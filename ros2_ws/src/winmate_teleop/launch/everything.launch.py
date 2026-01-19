from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8765',
        description='Port for Foxglove Bridge'
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': LaunchConfiguration('port')
        }]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )

    joy_to_wrench = Node(
        package='winmate_teleop',
        executable='joy_to_wrench',
        name='joy_to_wrench'
    )

    return LaunchDescription([
        port_arg,
        foxglove_bridge,
        joy_node,
        joy_to_wrench
    ])

