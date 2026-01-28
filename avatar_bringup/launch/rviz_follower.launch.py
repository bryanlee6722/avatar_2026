from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare('avatar_description')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf/follower', 'follower.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'avatar_model.rviz'])

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([
        # Joint State Publisher GUI 추가
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Trajectory -> JointState bridge
        Node(
            package='avatar_bringup',
            executable='traj_to_jointstate',
            name='traj_to_jointstate',
            output='screen',
            parameters=[
                {'traj_topic': '/arm_controller/joint_trajectory'},   # /leader/joint_trajectory로 변경
                {'joint_states_topic': '/joint_states'}
            ]
        ),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # rviz2 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
