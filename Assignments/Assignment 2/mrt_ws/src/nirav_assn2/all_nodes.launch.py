import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    d_rover1_node = Node(
        package='nirav_assn2',
        executable='d_rover1',
        name='d_rover1'
    )

    d_rover2_node = Node(
        package='nirav_assn2',
        executable='d_rover2',
        name='d_rover2'
    )

    d_rover3_node = Node(
        package='nirav_assn2',
        executable='d_rover3',
        name='d_rover3'
    )

    d_rover4_node = Node(
        package='nirav_assn2',
        executable='d_rover4',
        name='d_rover4'
    )

    basestation_node = Node(
        package='nirav_assn2',
        executable='basestation',
        name='basestation'
    )

    ld.add_action(d_rover1_node)
    ld.add_action(d_rover2_node)
    ld.add_action(d_rover3_node)
    ld.add_action(d_rover4_node)
    ld.add_action(basestation_node)

    return ld
