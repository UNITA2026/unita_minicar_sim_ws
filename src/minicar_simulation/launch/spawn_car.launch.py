import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():

    package_name = 'minicar_simulation'
    
    sim_pkg_share = get_package_share_directory(package_name)
    desc_pkg_share = get_package_share_directory('minicar_description')
    
    sim_share_path = os.path.dirname(sim_pkg_share)
    desc_share_path = os.path.dirname(desc_pkg_share)

    models_path = os.path.join(sim_pkg_share, 'models')

    set_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=sim_share_path + os.pathsep + desc_share_path + os.pathsep + models_path
    )

    print(f"Adding Model Path: {desc_share_path}")

    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y')
    z_arg = DeclareLaunchArgument('z', default_value='0.2', description='Spawn Z')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw')

    # URDF 파일 경로
    urdf_file = os.path.join(desc_pkg_share, 'urdf', 'unita_minicar.urdf')
    world_file = os.path.join(sim_pkg_share, 'worlds', 'unita_track.world')

    doc = xacro.process_file(urdf_file)
    robot_desc = doc.toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': True
                     }]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'car',
                   '-x', LaunchConfiguration('x'),  
                   '-y', LaunchConfiguration('y'),   
                   '-z', LaunchConfiguration('z'),   
                   '-Y', LaunchConfiguration('yaw')  
                   ],
        output='screen'
    )

    return LaunchDescription([
        x_arg, y_arg, z_arg, yaw_arg,
        set_model_path, 
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])