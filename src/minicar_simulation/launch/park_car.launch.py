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
    
    # [주의] YOLO 모델 파일 (.pt) 경로
    workspace_model_path = os.path.join(os.getcwd(), 'lane.pt')
    print(f"\n[Launch] Loading YOLO model from: {workspace_model_path}\n")

    obstacles = [
        {'name': 'cone_1', 'x': -0.862068, 'y': -3.225873, 'z': 0.0},
        {'name': 'cone_2', 'x': 0.542077, 'y': -3.225873, 'z': 0.0},
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
            'x': '-2.497558',  
            'y': '-2.281414',
            'z': '0.1',     
        }.items()
    )

    yolo_node = Node(
        package='camera_perception_pkg',
        executable='yolov8_node',
        name='yolov8_node',
        output='screen',
        parameters=[{
                'enable_lane': False,   # 차선 인식 끄기
                'enable_cone': True,    # 꼬깔 인식 켜기
        }],
        remappings=[
                ('camera1/image_raw', '/rear_camera/image_raw') 
        ]
    )

    yolo_viz_node = Node(
        package='debug_pkg',
        executable='yolov8_visualizer_node',
        name='yolov8_visualizer_node',
        output='screen',
        remappings=[
            ('camera1/image_raw', '/rear_camera/image_raw') 
        ]
    )

    bev_rear_node = Node(
        package='camera_perception_pkg',
        executable='bev_rear_node',
        name='bev_rear_node',
        output='screen'
    )

    parking_perception_node = Node(
        package='camera_perception_pkg',
        executable='parking_perception_node',
        name='parking_perception_node',
        output='screen'
    )

    parking_line_stop_node = Node(
        package='decision_making_pkg',
        executable='parking_line_stop_node',
        name='parking_line_stop_node',
        output='screen'
    )

    parking_planner_node = Node(
        package='decision_making_pkg',
        executable='parking_planner_node',
        name='parking_planner_node',
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
                yolo_viz_node,
                bev_rear_node,
                parking_perception_node,
                parking_line_stop_node,
            ]
        ),

        TimerAction(
            period=8.0,
            actions=[
                parking_planner_node,
                sim_sender_node
            ]
        )
    ])