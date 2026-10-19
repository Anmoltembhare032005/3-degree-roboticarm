import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

IGN_LIB = '/opt/ros/humble/lib'

def generate_launch_description():
    urdf_path = os.path.expanduser('~/three_dof_arm/urdf/three_dof_arm.urdf')
    with open(urdf_path) as f:
        robot_desc = f.read()

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
        output='screen',
        additional_env={
            'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': IGN_LIB,
            'LD_LIBRARY_PATH': IGN_LIB,
        })

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'three_dof_arm'],
        parameters=[{'use_sim_time': True}],
        output='screen',
        additional_env={
            'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': IGN_LIB,
            'LD_LIBRARY_PATH': IGN_LIB + ':/opt/ros/humble/opt',
        })

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')

    set_env = SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', IGN_LIB)

    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster'])
    arm = Node(package='controller_manager', executable='spawner',
               arguments=['arm_controller'])

    return LaunchDescription([
        set_env,
        gazebo,
        rsp,
        spawn,
        clock_bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
    ])
