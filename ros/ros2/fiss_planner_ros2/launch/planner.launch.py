"""Launch the FISS planner and (optionally) the trajectory controller.

ros2 launch fiss_planner_ros2 planner.launch.py odom:=/localization/odom
ros2 launch fiss_planner_ros2 planner.launch.py lanelet2_map:=/path/map.osm controller:=true rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration("params")
    args = [
        DeclareLaunchArgument(
            "params",
            default_value=PathJoinSubstitution(
                [FindPackageShare("fiss_planner_ros2"), "config", "planner.yaml"]
            ),
        ),
        DeclareLaunchArgument("odom", default_value="/odom"),
        DeclareLaunchArgument("obstacles", default_value="/perception/obstacles"),
        DeclareLaunchArgument("reference_path", default_value="/planning/reference_path"),
        DeclareLaunchArgument("goal", default_value="/goal_pose"),
        DeclareLaunchArgument("lanelet2_map", default_value=""),
        DeclareLaunchArgument("planner", default_value="fiss_plus"),
        DeclareLaunchArgument("controller", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
    ]
    planner = Node(
        package="fiss_planner_ros2",
        executable="planner_node",
        name="fiss_planner",
        output="screen",
        parameters=[
            params,
            {
                "map.lanelet2_path": LaunchConfiguration("lanelet2_map"),
                "planner.type": LaunchConfiguration("planner"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=[
            ("~/odom", LaunchConfiguration("odom")),
            ("~/obstacles", LaunchConfiguration("obstacles")),
            ("~/reference_path", LaunchConfiguration("reference_path")),
            ("~/goal", LaunchConfiguration("goal")),
        ],
    )
    controller = Node(
        package="fiss_planner_ros2",
        executable="controller_node",
        name="fiss_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("controller")),
        parameters=[params, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[("~/odom", LaunchConfiguration("odom")), ("~/trajectory", "/fiss_planner/trajectory")],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("fiss_planner_ros2"), "rviz", "fiss_planner.rviz"])],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    return LaunchDescription(args + [planner, controller, rviz])
