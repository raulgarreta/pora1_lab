from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")

    navigator_node = Node(
        package="autonomy_repo",
        executable="navigator.py",
        name="navigator_node",
        parameters=[{"use_sim_time": use_sim_time}]
    )


    rviz_goal_relay_node = Node(
        package="asl_tb3_lib",
        executable="rviz_goal_relay.py",
        parameters=[{"output_channel": "/cmd_nav"}]
    )

    state_publisher_node = Node(
        package="asl_tb3_lib",
        executable="state_publisher.py",

    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        navigator_node,
        rviz_goal_relay_node,
        state_publisher_node
    ])
