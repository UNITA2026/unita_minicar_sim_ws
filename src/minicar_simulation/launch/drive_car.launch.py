import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    
    sim_pkg_name = 'minicar_simulation'
    sim_pkg_share = get_package_share_directory(sim_pkg_name)
    
    obstacles = [
        {'name': 'cone_1', 'x': 2.817826, 'y': -1.352611, 'z': 0.0},
        {'name': 'cone_2', 'x': 1.915787, 'y': 1.855400, 'z': 0.0},
    ]

    spawn_cone_nodes = [
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f"spawn_{obs['name']}",
            arguments=[
                '-entity', obs['name'],
                '-database', 'construction_cone',
                '-x', str(obs['x']),
                '-y', str(obs['y']),
                '-z', str(obs['z'])
            ],
            output='screen'
        )
        for obs in obstacles
    ]

    spawn_car_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_share, 'launch', 'spawn_car.launch.py')
        ),
        launch_arguments={
            'x': '1.126491',  
            'y': '5.710308',
            'z': '0.1',     
            'yaw': '2.929481'
        }.items()
    )

    yolo_node = Node(
        package='camera_perception_pkg',
        executable='yolov8_node',
        name='yolov8_node',
        output='screen'
    )

    yolo_visualizer_node = Node(
        package='debug_pkg',
        executable='yolov8_visualizer_node',
        name='yolov8_visualizer_node',
        output='screen'
    )

    box_lidar_match_node = Node(
        package='camera_perception_pkg',
        executable='box_lidar_match_node',
        name='box_lidar_match_node',
        output='screen'
    )

    lane_info_node = Node(
        package='camera_perception_pkg',
        executable='lane_info_extractor_node',
        name='lane_info_extractor_node',
        output='screen'
    )

    path_planner_node = Node(
        package='decision_making_pkg',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen'
    )

    motion_planner_node = Node(
        package='decision_making_pkg',
        executable='motion_planner_node',
        name='motion_planner_node',
        output='screen'
    )

    sim_sender_node = Node(
        package='minicar_simulation', 
        executable='simulation_sender.py', 
        name='simulation_sender_node',
        output='screen'
    )
    

    return LaunchDescription([
        spawn_car_launch,
        
        TimerAction(
            period=1.0,
            actions=spawn_cone_nodes
        ),
        
        TimerAction(
            period=2.0,
            actions=[
                yolo_node,
                yolo_visualizer_node,
                box_lidar_match_node,
                lane_info_node,
                path_planner_node,
                motion_planner_node,
                sim_sender_node,
            ]
        )
    ])
