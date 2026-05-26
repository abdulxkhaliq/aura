from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("so_arm101", package_name="aura_moveit_config")
        .planning_pipelines(pipelines=["stomp", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        output="screen",
    )

    from launch.actions import TimerAction
    from launch_ros.actions import Node as SpawnerNode
    from launch.actions import ExecuteProcess

    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner", "arm_controller"],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        move_group_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])
